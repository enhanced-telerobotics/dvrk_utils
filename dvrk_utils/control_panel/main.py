"""CLI and cooperative ROS / HTTP / GTK event loops."""

import argparse
import asyncio
import ipaddress
import json
import os
from pathlib import Path
import re
import signal
import sys

from .configuration import advertised_hostname, load_config


def parse_options(arguments):
    """Parse panel options after the caller removes ROS arguments."""
    parser = argparse.ArgumentParser(description='dVRK Web Control Panel')
    parser.add_argument('-c', '--config', help='JSON config (default: ~/.config/dvrk_utils/control_panel.json)')
    parser.add_argument('-C', '--console', help='Console namespace (default: console)')
    parser.add_argument('--host', default=None, help='Listen address (default: all IPv4 interfaces)')
    parser.add_argument('--port', type=int, default=None, help='HTTP port (default: 8080)')
    parser.add_argument('--advertise-address', help='PC IP address to use in the QR code')
    parser.add_argument('--hostname', help='Hostname for the link and QR code (overrides config)')
    parser.add_argument('--headless', action='store_true', help='Print connection links without a GTK window')
    parser.add_argument('--reset-pairing', action='store_true',
                        help='Replace the saved pairing token and invalidate previous links')
    options = parser.parse_args(arguments)
    try:
        config = load_config(options.config)
        options.hostname = advertised_hostname(
            options.hostname if options.hostname is not None else config.get('hostname', ''))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    options.host = options.host if options.host is not None else config.get('host', '0.0.0.0')
    options.port = options.port if options.port is not None else config.get('port', 8080)
    if type(options.port) is not int or not 1 <= options.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    for value in (options.host, options.advertise_address):
        if value:
            try:
                ipaddress.ip_address(value)
            except (ValueError, TypeError):
                parser.error('Listen and advertised addresses must be IP addresses')
    options.console = options.console if options.console is not None else config.get('console', 'console')
    if (not isinstance(options.console, str)
            or not re.fullmatch(r'/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*/?', options.console)):
        parser.error('console must be a valid, nonempty ROS namespace')
    options.console = options.console.strip('/')
    if options.advertise_address:
        address = ipaddress.ip_address(options.advertise_address)
        if address.is_unspecified or address.is_multicast:
            parser.error('advertise-address must be a unicast address')
        if options.host not in ('0.0.0.0', '::') and options.host != options.advertise_address:
            parser.error('advertise-address must match the specific listen address')
        if options.host == '0.0.0.0' and address.version != 4:
            parser.error('an IPv4 listen address requires an IPv4 advertised address')
    return options


async def run(options):
    """Run HTTP, ROS callbacks, reset sequences, and optional GTK cooperatively."""
    from aiohttp import web
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    from .bridge import RosBridge
    from .connection_window import ConnectionWindow, connection_url, network_addresses
    from .server import PanelServer
    from .pairing import PairingStore

    # Prefer installed assets, with a source-tree fallback for development.
    source_root = Path(__file__).resolve().parents[2] / 'share' / 'control_panel'
    web_root = source_root if source_root.is_dir() else (
        Path(get_package_share_directory('dvrk_utils')) / 'control_panel')
    if not (web_root / 'index.html').is_file():
        raise RuntimeError(f'Web assets not found: {web_root}')
    pairing = PairingStore()
    token = pairing.reset() if options.reset_pairing else pairing.load()
    node = RosBridge(options.console)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    server = PanelServer(node, token, web_root, token_factory=pairing.reset)
    runner = web.AppRunner(server.app, access_log=None)
    window = None
    regeneration = None
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, stopped.set)
    try:
        await runner.setup()
        await web.TCPSite(runner, options.host, options.port).start()
        addresses = network_addresses(options.host, options.advertise_address, options.hostname)
        print('dVRK web control panel is running. Scan the QR code or open a link:')
        for interface, address in addresses:
            print(f'  {interface}: {connection_url(address, options.port, token)}')
        print('Choose Take control in the browser. Ctrl+C stops the server.', flush=True)

        async def regenerate_connection():
            window.regenerate_button.set_sensitive(False)
            try:
                new_token = await server.regenerate_token()
                window.set_token(new_token)
                print('Connection regenerated. Previous links and sessions are revoked:')
                for interface, address in addresses:
                    print(f'  {interface}: {connection_url(address, options.port, new_token)}')
                print('Scan the new PC QR code to connect.', flush=True)
            except OSError as error:
                print(f'Unable to save new pairing token: {error}. '
                      'Previous connection remains active.', file=sys.stderr)
            finally:
                window.regenerate_button.set_sensitive(True)

        def request_regeneration():
            nonlocal regeneration
            if regeneration is None or regeneration.done():
                regeneration = asyncio.create_task(regenerate_connection())

        if not options.headless:
            os.environ['GTK_IM_MODULE'] = 'gtk-im-context-simple'
            try:
                window = ConnectionWindow(
                    addresses, options.port, token, stopped.set, request_regeneration)
            except (ImportError, RuntimeError, ValueError) as error:
                print(f'Connection window unavailable: {error}', file=sys.stderr)
        while not stopped.is_set() and rclpy.ok():
            # Bound ROS work so a busy ROS graph cannot monopolize the web UI.
            for _ in range(10):
                executor.spin_once(timeout_sec=0)
            node.tick()
            if window:
                window.tick(len(server.clients), server.owner_session is not None,
                            server.owner is not None)
            await asyncio.sleep(0.02)
    finally:
        if regeneration is not None:
            await regeneration
        if window:
            window.close()
        await runner.cleanup()
        executor.shutdown()
        node.destroy_node()
        for signum in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(signum)


def main(arguments=None):
    """Initialize ROS and launch the service with actionable startup errors."""
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.utilities import remove_ros_args
        import aiohttp  # noqa: F401
    except ImportError as error:
        print(f'Missing dependency: {error}. Source ROS and install package dependencies '
              '(python3-aiohttp, python3-qrcode, python3-pil, python3-gi).', file=sys.stderr)
        return 1
    arguments = sys.argv if arguments is None else ['control_panel', *arguments]
    options = parse_options(remove_ros_args(args=arguments)[1:])
    rclpy.init(args=arguments, signal_handler_options=SignalHandlerOptions.NO)
    try:
        asyncio.run(run(options))
    except (OSError, RuntimeError) as error:
        print(f'Unable to run web control panel: {error}', file=sys.stderr)
        return 1
    finally:
        rclpy.try_shutdown()
    return 0
