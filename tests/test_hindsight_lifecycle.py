import subprocess
import threading
import unittest
from unittest.mock import Mock, patch

from robot_ui.hindsight_lifecycle import HindsightLifecycle, safe_failure


class LifecycleTests(unittest.TestCase):
    def test_observer_never_starts_or_stops_container(self):
        done = threading.Event()
        service = HindsightLifecycle(changed=lambda *_: done.set())
        with patch.object(service, 'inspect', return_value='on'), patch.object(service, '_execute') as execute:
            service.start()
            self.assertTrue(done.wait(2))
            self.assertTrue(service.wait_until_ready())
            service.stop()
            service._thread.join(2)
        execute.assert_not_called()

    def test_real_states(self):
        for raw, expected in [(b'"running" "healthy"', 'on'),
                              (b'"running" "starting"', 'starting'),
                              (b'"exited" "unhealthy"', 'off')]:
            service = HindsightLifecycle()
            with patch.object(service, '_execute', side_effect=[b'abc', raw]):
                self.assertEqual(service.inspect(), expected)
        with patch.object(service, '_execute', return_value=b''):
            self.assertEqual(service.inspect(), 'off')
        with patch.object(service, '_execute', side_effect=[b'abc', b'"running" "unhealthy"']):
            with self.assertRaises(RuntimeError):
                service.inspect()

    def test_actions_verified_and_duplicate_clicks_rejected(self):
        for enabled, expected in [(True, 'on'), (False, 'off')]:
            service = HindsightLifecycle()
            service._publish('off' if enabled else 'on')
            self.assertTrue(service.set_enabled(enabled))
            self.assertFalse(service.set_enabled(enabled))
            service.changed = lambda *_: service.stop()
            with patch.object(service, '_execute') as execute, patch.object(service, 'inspect', return_value=expected):
                service._run()
            self.assertEqual(service.state, expected)
            self.assertEqual(execute.call_args.args[0][6], 'up' if enabled else 'stop')

    def test_failed_action_reports_error_and_observed_state(self):
        changes = []
        service = HindsightLifecycle(report=Mock())
        service._publish('off')
        service.set_enabled(True)
        def changed(state, error):
            changes.append((state, error))
            service.stop()
        service.changed = changed
        with patch.object(service, '_execute', side_effect=RuntimeError('command-timeout')), \
             patch.object(service, 'inspect', return_value='off'):
            service._run()
        self.assertEqual(changes, [('off', 'command-timeout')])
        self.assertFalse(service.wait_until_ready())

    def test_docker_errors_are_sanitized(self):
        service = HindsightLifecycle()
        def failed(*args, **kwargs):
            kwargs['stderr'].write(b'permission denied sk-secret')
            return Mock(returncode=1)
        with patch('robot_ui.hindsight_lifecycle.subprocess.run', side_effect=failed):
            with self.assertRaisesRegex(RuntimeError, 'docker-permission') as caught:
                service._execute(['docker'])
        self.assertNotIn('sk-secret', str(caught.exception))
        for exception, category in [(FileNotFoundError(), 'docker-exec'),
                                     (subprocess.TimeoutExpired('docker', 15, output=b'secret'), 'command-timeout')]:
            with patch('robot_ui.hindsight_lifecycle.subprocess.run', side_effect=exception):
                with self.assertRaisesRegex(RuntimeError, category):
                    service._execute(['docker'])
        self.assertNotIn('secret', safe_failure(b'unknown secret'))

    def test_close_releases_readiness_and_prevents_start(self):
        service = HindsightLifecycle()
        service.stop()
        service.start()
        self.assertIsNone(service._thread)
        self.assertFalse(service.wait_until_ready())
        self.assertFalse(service.set_enabled(True))
