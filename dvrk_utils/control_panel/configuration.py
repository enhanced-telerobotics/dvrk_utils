"""User configuration and advertised hostname for the control panel."""

import json
import os
from pathlib import Path
import re
import socket
import subprocess


def config_directory():
    value = os.environ.get('XDG_CONFIG_HOME')
    root = Path(value) if value and Path(value).is_absolute() else Path.home() / '.config'
    return root / 'dvrk_utils'


def load_config(filename=None):
    """Create an editable default config; explicit paths must already exist."""
    path = Path(filename).expanduser() if filename else config_directory() / 'control_panel.json'
    if filename is None:
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with path.open('x') as output:
                json.dump({'hostname': '', 'host': '0.0.0.0', 'port': 8080,
                           'console': 'console'}, output, indent=2)
                output.write('\n')
        except FileExistsError:
            pass
    config = json.loads(path.read_text())
    if not isinstance(config, dict):
        raise ValueError(f'{path}: configuration must be a JSON object')
    return config


def advertised_hostname(configured):
    """Use a configured name, otherwise the first hostname -a alias or PC name."""
    if not isinstance(configured, str):
        raise ValueError('hostname must be a string')
    name = configured.strip()
    if not name:
        try:
            result = subprocess.run(['hostname', '-a'], capture_output=True,
                                    text=True, check=True, timeout=2)
            aliases = result.stdout.split()
        except (OSError, subprocess.SubprocessError):
            aliases = []
        name = aliases[0] if aliases else socket.gethostname()
    labels = name.rstrip('.').split('.')
    if (len(name) > 253 or not all(re.fullmatch(
            r'[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?', label)
            for label in labels)):
        raise ValueError('hostname must be a hostname without a scheme, port, or path')
    return name
