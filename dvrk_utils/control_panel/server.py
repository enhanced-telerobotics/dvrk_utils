"""Authenticated WebSocket API with exclusive, explicitly released ownership."""

import asyncio
from collections import deque
from dataclasses import dataclass, field
import hmac
import json
from pathlib import Path
import secrets
import time

from aiohttp import WSMsgType, web

from .bridge import CommandError


@dataclass(eq=False)
class Client:
    """Socket with heartbeat, rate, and duplicate-request tracking."""
    socket: web.WebSocketResponse
    session: str = field(default_factory=lambda: secrets.token_urlsafe(32))
    last_seen: float = field(default_factory=time.monotonic)
    commands: deque = field(default_factory=deque)
    request_ids: deque = field(default_factory=lambda: deque(maxlen=128))


class PanelServer:
    """Serve the static panel and arbitrate one active browser controller."""

    def __init__(self, bridge, token, web_root, token_factory=None):
        """Attach explicit asset and socket routes to an aiohttp application."""
        self.bridge = bridge
        self.token = token
        self._token_factory = token_factory or (lambda: secrets.token_urlsafe(32))
        self.web_root = Path(web_root)
        self.clients = set()
        self.owner = None
        self.owner_session = None
        self._sockets = set()
        self._connections = 0
        self._ticker = None
        self.app = web.Application(client_max_size=4096, middlewares=[self._headers])
        self.app.router.add_get('/', self._index)
        self.app.router.add_get('/panel.css', self._asset)
        self.app.router.add_get('/panel.js', self._asset)
        self.app.router.add_get('/ws', self._websocket)
        self.app.on_startup.append(self._startup)
        self.app.on_shutdown.append(self._shutdown)

    @web.middleware
    async def _headers(self, request, handler):
        response = await handler(request)
        response.headers.update({
            'Cache-Control': 'no-store',
            'Referrer-Policy': 'no-referrer',
            'X-Content-Type-Options': 'nosniff',
            'X-Frame-Options': 'DENY',
            'Content-Security-Policy': (
                "default-src 'self'; script-src 'self'; style-src 'self'; "
                "connect-src 'self'; img-src 'self'; frame-ancestors 'none'; "
                "base-uri 'none'; form-action 'none'"),
        })
        return response

    async def _index(self, request):
        return web.FileResponse(self.web_root / 'index.html')

    async def _asset(self, request):
        return web.FileResponse(self.web_root / request.path.lstrip('/'))

    async def _startup(self, app):
        self._ticker = asyncio.create_task(self._broadcast_loop())

    async def _shutdown(self, app):
        if self._ticker:
            self._ticker.cancel()
            await asyncio.gather(self._ticker, return_exceptions=True)
        self.owner = None
        self.owner_session = None
        await asyncio.gather(*(
            socket.close(code=1001, message=b'Server stopping')
            for socket in list(self._sockets)), return_exceptions=True)

    async def regenerate_token(self):
        """Revoke all sessions and return a fresh PC-issued pairing token."""
        # Save before revoking sessions; a disk failure preserves the current link.
        self.token = self._token_factory()
        self.owner = None
        self.owner_session = None
        self.clients.clear()
        # Revocation precedes the first await, so queued old commands are rejected.
        await asyncio.gather(*(
            socket.close(code=1008, message=b'Connection token regenerated on PC')
            for socket in list(self._sockets)), return_exceptions=True)
        return self.token

    async def _websocket(self, request):
        # The pairing secret lives in a URL fragment, then the first WS message;
        # it never appears in an HTTP request URL or access log.
        origin = request.headers.get('Origin')
        if origin != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden(text='Open the control panel on this server.')
        if self._connections >= 8:
            raise web.HTTPServiceUnavailable(text='Too many connected devices.')
        self._connections += 1
        ws = web.WebSocketResponse(max_msg_size=4096, heartbeat=5, timeout=2)
        self._sockets.add(ws)
        client = Client(ws)
        try:
            await ws.prepare(request)
            message = await asyncio.wait_for(ws.receive(), timeout=5)
            try:
                auth = json.loads(message.data) if message.type == WSMsgType.TEXT else {}
                token = auth.get('token') if isinstance(auth, dict) else None
            except (ValueError, TypeError):
                token = None
            if not isinstance(token, str) or not hmac.compare_digest(
                    token.encode(), self.token.encode()):
                await ws.close(code=1008, message=b'Invalid pairing token')
                return ws
            session = auth.get('session')
            if (isinstance(session, str) and self.owner_session
                    and hmac.compare_digest(session.encode(), self.owner_session.encode())):
                if self.owner is not None:
                    # A duplicated tab cannot impersonate an active controller.
                    await ws.close(code=4001, message=b'Control session already connected')
                    return ws
                client.session = session
                self.owner = client
            client.last_seen = time.monotonic()
            self.clients.add(client)
            await self._send_state(client)
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    if message.type == WSMsgType.ERROR:
                        break
                    continue
                await self._handle_message(client, message.data)
        except (asyncio.TimeoutError, ConnectionError):
            await ws.close()
        finally:
            self.clients.discard(client)
            self._sockets.discard(ws)
            self._connections -= 1
            if self.owner is client:
                self.owner = None
        return ws

    async def _handle_message(self, client, raw):
        if client not in self.clients:
            await client.socket.close(code=1008, message=b'Connection revoked')
            return
        request_id = None
        try:
            message = json.loads(raw)
            if not isinstance(message, dict):
                raise CommandError('Expected a JSON object.')
            kind = message.get('type')
            if kind == 'ping':
                client.last_seen = time.monotonic()
                return
            request_id = message.get('id')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 80:
                raise CommandError('A request ID is required.')
            if request_id in client.request_ids:
                raise CommandError('Duplicate request; it was not executed again.')
            client.request_ids.append(request_id)
            now = time.monotonic()
            client.last_seen = now
            while client.commands and now - client.commands[0] > 1:
                client.commands.popleft()
            if len(client.commands) >= 12:
                raise CommandError('Too many requests; try again shortly.')
            client.commands.append(now)
            if kind == 'acquire':
                if self.owner_session and self.owner is not client:
                    raise CommandError('Control is reserved by another device. '
                                       'It must release control, or regenerate the QR on the PC.')
                self.owner = client
                self.owner_session = client.session
            elif kind == 'release':
                if self.owner is client:
                    self.owner = None
                    self.owner_session = None
            elif kind == 'command':
                if self.owner is not client:
                    raise CommandError('Take control before sending commands.')
                self.bridge.execute(message)
            else:
                raise CommandError('Unknown request type.')
            await client.socket.send_json({
                'type': 'ack', 'id': request_id, 'ok': True,
                'message': 'Request sent; waiting for ROS feedback.'
                if kind == 'command' else 'Control updated.',
            })
        except (ValueError, TypeError) as error:
            await client.socket.send_json({
                'type': 'ack', 'id': request_id, 'ok': False, 'message': str(error),
            })
        await self._send_state(client)

    async def _send_state(self, client):
        await asyncio.wait_for(client.socket.send_json({
            'type': 'state', 'state': self.bridge.snapshot(),
            'session': client.session,
            'control': 'mine' if self.owner is client else (
                'busy' if self.owner_session else 'available'),
            'clients': len(self.clients),
        }), timeout=1)

    async def _broadcast_loop(self):
        while True:
            await asyncio.sleep(0.25)
            clients = list(self.clients)
            results = await asyncio.gather(*(
                self._send_state(client) for client in clients), return_exceptions=True)
            for client, result in zip(clients, results):
                if isinstance(result, Exception) or time.monotonic() - client.last_seen > 6:
                    # Drop stalled sockets; ownership remains reserved for that session.
                    self.clients.discard(client)
                    if self.owner is client:
                        self.owner = None
                    asyncio.create_task(client.socket.close())
