"""Own the entire ROS launch process group, with a bounded, readable log."""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path


class TrackingProcess:
    def __init__(self):
        self.process = None
        self.log = None
        self.path = None
        self.offset = 0

    def start(self, command):
        if self.process is not None:
            raise RuntimeError('Tracking process already owned; stop before restarting')
        self.offset = 0
        self.log = tempfile.NamedTemporaryFile(prefix='tracking-', suffix='.log', delete=False)
        self.path = self.log.name
        try:
            self.process = subprocess.Popen(
                ['bash', '-c', command], stdout=self.log, stderr=subprocess.STDOUT,
                start_new_session=True)
        except Exception:
            self.log.close()
            raise

    def read_log(self):
        if not self.path:
            return ''
        with open(self.path, 'r', errors='replace') as stream:
            stream.seek(self.offset)
            text = stream.read(65536)
            self.offset = stream.tell()
            return text

    def _group_alive(self):
        # Linux /proc also distinguishes exited children awaiting reaping.
        for entry in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = entry.read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == self.process.pid and fields[0] != 'Z':
                    return True
            except (OSError, ValueError, IndexError):
                continue
        return False

    def stop(self):
        if self.process is not None:
            # Signal the group even if its leader has exited.
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + (5 if sig == signal.SIGINT else 2)
                while self._group_alive() and time.monotonic() < deadline:
                    self.process.poll()
                    time.sleep(.02)
                if not self._group_alive():
                    break
            self.process.wait(timeout=2)
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None
