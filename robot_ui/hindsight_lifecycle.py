"""Observe and control Docker; UI shutdown never stops the service."""

from pathlib import Path
import subprocess
import tempfile
import threading
import json


def safe_failure(output):
    """Map untrusted Docker output to fixed messages; never echo any of it."""
    text = output.lower()
    if b'permission denied' in text or b'operation not permitted' in text:
        return 'docker-permission: user chạy UI không có quyền truy cập Docker.'
    if any(term in text for term in (b'cannot connect', b'is the docker daemon running',
                                     b'error during connect', b'connection refused')):
        return 'docker-daemon: không kết nối được Docker Engine.'
    if b'unknown flag' in text and b'--wait' in text:
        return 'compose-wait-unsupported: cần Compose hỗ trợ --wait và --wait-timeout.'
    if any(term in text for term in (b'required variable', b'missing mandatory value',
                                     b'invalid interpolation', b'must be a mapping',
                                     b'validating ', b'yaml:')):
        return 'compose-config: kiểm tra biến bắt buộc trong .env và cấu hình Compose.'
    if b'unhealthy' in text or b'healthcheck' in text:
        return 'container-unhealthy: kiểm tra healthcheck và log service trên máy chạy UI.'
    if b'timeout' in text or b'timed out' in text:
        return 'health-or-start-timeout: quá thời hạn; kiểm tra health, tải image và provider.'
    if b'port is already allocated' in text or b'address already in use' in text:
        return 'port-in-use: kiểm tra cổng TCP 8888/9999 hoặc cổng đã cấu hình.'
    if b'no such container' in text:
        return 'container-missing: container không tồn tại.'
    return 'docker-command-failed: kiểm tra Docker và log service trên máy chạy UI.'


def _diagnosis(capture):
    capture.seek(0)
    return safe_failure(capture.read(128 * 1024))


class HindsightLifecycle:
    START_TIMEOUT = 300
    POLL_SECONDS = 5

    def __init__(self, report=print, changed=None):
        self.root = Path(__file__).resolve().parents[1]
        self.report = report
        self.changed = changed or (lambda state, error: None)
        self._closing = threading.Event()
        self._wake = threading.Event()
        self._settled = threading.Event()
        self._lock = threading.RLock()
        self._thread = None
        self._pending = None
        self.state = 'checking'
        self._command = ['docker', 'compose', '--project-directory', str(self.root),
                         '-f', str(self.root / 'docker-compose.hindsight.yml')]

    def _execute(self, command, timeout=15):
        # Never return stderr, inspect environment, or emit raw Docker output.
        with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as error:
            try:
                result = subprocess.run(command, cwd=self.root, stdout=output,
                                        stderr=error, timeout=timeout, check=False,
                                        start_new_session=True)
            except subprocess.TimeoutExpired:
                raise RuntimeError('command-timeout') from None
            except OSError:
                raise RuntimeError('docker-exec') from None
            if result.returncode:
                raise RuntimeError(_diagnosis(error))
            output.seek(0)
            return output.read(128 * 1024)

    def inspect(self):
        ids = self._execute(['docker', 'ps', '-a', '--filter', 'name=^/hindsight$',
                             '--format', '{{.ID}}']).strip()
        if not ids:
            return 'off'
        raw = self._execute(['docker', 'inspect', '--format',
                             '{{json .State.Status}} {{if .State.Health}}{{json .State.Health.Status}}{{else}}"unknown"{{end}}',
                             'hindsight'])
        try:
            status, health = [json.loads(part) for part in raw.decode().split()]
        except (ValueError, UnicodeError):
            raise RuntimeError('docker-state-invalid') from None
        if status in ('exited', 'created'):
            return 'off'
        if status == 'running' and health == 'healthy':
            return 'on'
        if status in ('running', 'restarting') and health != 'unhealthy':
            return 'starting'
        raise RuntimeError('container-unhealthy-or-unavailable')

    def _publish(self, state, error=''):
        with self._lock:
            self.state = state
            if state in ('checking', 'starting', 'stopping'):
                self._settled.clear()
            else:
                self._settled.set()
        if not self._closing.is_set():
            self.changed(state, error)
            if error:
                self.report('[Chế độ học] ' + error)

    def start(self):
        if self._thread is None and not self._closing.is_set():
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def set_enabled(self, enabled):
        with self._lock:
            if self._closing.is_set() or self._pending is not None or self.state in (
                    'checking', 'starting', 'stopping'):
                return False
            self._pending = bool(enabled)
            self.state = 'starting' if enabled else 'stopping'
            self._settled.clear()
        self.changed(self.state, '')
        self._wake.set()
        return True

    def _run(self):
        while not self._closing.is_set():
            with self._lock:
                action = self._pending
            error = ''
            if action is not None:
                try:
                    args = (['up', '-d', '--wait', '--wait-timeout', '120', 'hindsight']
                            if action else ['stop', '--timeout', '15', 'hindsight'])
                    self._execute(self._command + args, self.START_TIMEOUT if action else 25)
                except RuntimeError as exc:
                    error = str(exc)
            try:
                state = self.inspect()
            except RuntimeError as exc:
                state, error = 'error', error or str(exc)
            with self._lock:
                # A click can arrive during a periodic inspect. Preserve that
                # pending action and its busy state until the next iteration.
                interrupted = action is None and self._pending is not None
                if action is not None:
                    self._pending = None
                if not interrupted:
                    self._publish(state, error)
            self._wake.wait(self.POLL_SECONDS)
            self._wake.clear()

    def wait_until_ready(self):
        self._settled.wait(self.START_TIMEOUT + 10)
        return self.state == 'on' and not self._closing.is_set()

    def stop(self):
        """Detach the observer without changing Docker state or blocking Qt."""
        self._closing.set()
        self._settled.set()
        self._wake.set()
