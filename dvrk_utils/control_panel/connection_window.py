"""Small local GTK window showing selectable connection addresses and QR codes."""

import io
import ipaddress
import json
import socket
import subprocess


def network_addresses(bind_host, advertised=None, hostname=None):
    """List interface addresses, respecting an explicit listen address."""
    if advertised:
        return [('Selected address', advertised)]
    if bind_host not in ('0.0.0.0', '::'):
        return [('Server address', bind_host)]
    addresses = [('Hostname', hostname)] if hostname else []
    try:
        result = subprocess.run(
            ['ip', '-j', '-4', 'address', 'show', 'up'], capture_output=True,
            text=True, check=True, timeout=2)
        for interface in json.loads(result.stdout):
            for address in interface.get('addr_info', []):
                value = address.get('local', '')
                if address.get('scope') == 'global' and value:
                    addresses.append((interface['ifname'], value))
    except (OSError, ValueError, subprocess.SubprocessError):
        try:
            infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
        except OSError:
            infos = []
        for info in infos:
            value = info[4][0]
            if not ipaddress.ip_address(value).is_loopback:
                addresses.append(('Network', value))
    unique = list(dict.fromkeys(addresses))
    return unique or [('Local only', '127.0.0.1')]


def connection_url(address, port, token):
    """Encode the pairing secret in the fragment, including IPv6 brackets."""
    host = f'[{address}]' if ':' in address else address
    return f'http://{host}:{port}/#token={token}'


class ConnectionWindow:
    """Show the network selector, pairing QR, and current client count."""

    def __init__(self, addresses, port, token, on_close, on_regenerate):
        """Create GTK widgets on the thread that will call tick()."""
        import gi
        gi.require_version('Gtk', '3.0')
        gi.require_version('GdkPixbuf', '2.0')
        from gi.repository import GdkPixbuf, GLib, Gtk
        import qrcode

        initialized, _ = Gtk.init_check([])
        if not initialized:
            raise RuntimeError('No graphical display; use the printed connection URL.')
        self.GLib = GLib
        self.token = token
        self.window = Gtk.Window(title='dVRK Web Control Panel')
        self.window.set_default_size(430, 500)
        self.window.set_border_width(24)

        def request_close(*_):
            on_close()
            return True

        self.window.connect('delete-event', request_close)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)
        self.window.add(box)
        title = Gtk.Label()
        title.set_markup('<span size="x-large" weight="bold">dVRK Web Control Panel</span>')
        box.pack_start(title, False, False, 0)
        instructions = Gtk.Label(
            label='Connect your device and PC to the same network.\n'
                  'Choose the PC network address, then scan to connect.')
        instructions.set_line_wrap(True)
        box.pack_start(instructions, False, False, 0)
        selector = Gtk.ComboBoxText()
        self.selector = selector
        for interface, address in addresses:
            selector.append_text(f'{interface} · {address}')
        box.pack_start(selector, False, False, 0)
        image = Gtk.Image()
        box.pack_start(image, True, True, 0)
        address_label = Gtk.Label()
        address_label.set_selectable(True)
        box.pack_start(address_label, False, False, 0)
        self.status = Gtk.Label(label='Server running · 0 devices connected')
        box.pack_start(self.status, False, False, 0)
        copy_button = Gtk.Button(label='Copy connection link')
        box.pack_start(copy_button, False, False, 0)
        self.regenerate_button = Gtk.Button(label='Regenerate connection / release control')
        self.regenerate_button.connect('clicked', lambda *_: on_regenerate())
        box.pack_start(self.regenerate_button, False, False, 0)

        def update_qr(combo):
            address = addresses[combo.get_active()][1]
            self.url = connection_url(address, port, self.token)
            code = qrcode.QRCode(box_size=6, border=4)
            code.add_data(self.url)
            code.make(fit=True)
            output = io.BytesIO()
            code.make_image(fill_color='black', back_color='white').save(output, format='PNG')
            loader = GdkPixbuf.PixbufLoader.new_with_type('png')
            loader.write(output.getvalue())
            loader.close()
            image.set_from_pixbuf(loader.get_pixbuf())
            address_label.set_text(self.url.split('#')[0])

        def copy_link(_):
            from gi.repository import Gdk
            Gtk.Clipboard.get(Gdk.SELECTION_CLIPBOARD).set_text(self.url, -1)

        selector.connect('changed', update_qr)
        self._update_qr = update_qr
        selector.set_active(0)
        copy_button.connect('clicked', copy_link)
        self.window.show_all()

    def set_token(self, token):
        """Refresh the QR and copied link while keeping the selected interface."""
        self.token = token
        self._update_qr(self.selector)

    def tick(self, clients, owner, owner_connected=True):
        """Update status and process a bounded slice of GTK events."""
        # GTK and ROS run in short, nonblocking slices on the asyncio thread.
        self.status.set_text(f'Server running · {clients} devices connected\n'
                             + ('Control in use' if owner and owner_connected else
                                'Control reserved · reconnect or regenerate' if owner else
                                'Control available'))
        context = self.GLib.MainContext.default()
        for _ in range(20):
            if not context.pending():
                break
            context.iteration(False)

    def close(self):
        """Destroy the local connection window."""
        self.window.destroy()
