"""Persist the PC pairing secret across control panel restarts."""

from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import re
import secrets
import tempfile

from .configuration import config_directory


class PairingStore:
    """Keep a private token in the user's XDG configuration directory."""

    def __init__(self):
        self.path = config_directory() / 'control_panel.token'

    @contextmanager
    def _locked(self):
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd = os.open(str(self.path) + '.lock', os.O_CREAT | os.O_RDWR, 0o600)
        with os.fdopen(fd, 'w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def _write(self):
        token = secrets.token_urlsafe(32)
        fd, name = tempfile.mkstemp(prefix='.pairing-', dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w') as output:
                output.write(token + '\n')
                output.flush()
                os.fsync(output.fileno())
            os.replace(name, self.path)
        finally:
            Path(name).unlink(missing_ok=True)
        return token

    def load(self):
        """Create once; refuse corrupt files instead of silently changing links."""
        with self._locked():
            try:
                token = self.path.read_text().strip()
            except FileNotFoundError:
                return self._write()
            if not re.fullmatch(r'[A-Za-z0-9_-]{43}', token):
                raise RuntimeError(f'Invalid pairing token in {self.path}; '
                                   'use --reset-pairing to replace it.')
            self.path.chmod(0o600)
            return token

    def reset(self):
        """Atomically replace the saved token only on explicit request."""
        with self._locked():
            return self._write()
