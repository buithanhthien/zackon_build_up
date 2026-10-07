"""Own a mapping subprocess group without touching other robot processes."""
import os
import signal
import subprocess
import tempfile
import time


class MappingProcess:
    def __init__(self, command):
        self.output = tempfile.TemporaryFile(mode='w+b')
        try:
            self.process = subprocess.Popen(
                ['bash', '-c', command], start_new_session=True,
                stdout=self.output, stderr=subprocess.STDOUT,
            )
        except Exception:
            self.output.close()
            raise
        self.group_id = self.process.pid

    def poll(self):
        return self.process.poll()

    def error_detail(self):
        self.output.seek(0, os.SEEK_END)
        self.output.seek(max(0, self.output.tell() - 6000))
        return self.output.read().decode('utf-8', errors='replace')[-1500:].strip()

    def stop(self):
        # A launch parent may have exited while its ROS children still run.
        # Signal the owned group even when poll() reports a finished parent.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(self.group_id, sig)
            except ProcessLookupError:
                break
            try:
                self.process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                continue
            if sig != signal.SIGKILL:
                # Allow remaining children to shut down after their parent exits.
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    try:
                        os.killpg(self.group_id, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.02)
        self.process.wait(timeout=1)
        self.output.close()
