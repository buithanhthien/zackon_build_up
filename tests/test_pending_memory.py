import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from robot_ui.correction_memory import CorrectionMemory, MemoryUnavailable
from robot_ui.pending_memory import PendingMemory


def client(action, fact='', answer=''):
    result = Mock()
    result.chat.completions.create.return_value = SimpleNamespace(choices=[
        SimpleNamespace(message=SimpleNamespace(content=json.dumps({
            'action': action, 'fact': fact, 'answer': answer, 'document_id': '',
            'provenance': 'user', 'derivation': 'stated'})))])
    return result


class PendingMemoryTests(unittest.TestCase):
    def setUp(self):
        env = patch.dict(os.environ, {'HINDSIGHT_ENABLED': 'true'}, clear=True)
        env.start()
        self.addCleanup(env.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / 'private' / 'pending.sqlite3'

    def teach_with_timeout(self):
        memory = CorrectionMemory(storage_path=self.path)
        with patch.object(memory, 'recall', return_value=[]), patch.object(
                memory, 'retain', side_effect=MemoryUnavailable(category='timeout')):
            answer = memory.respond(client('correct', fact='Robot TEST uses a blue battery.'),
                                    'test', 'Robot TEST uses a blue battery.', '', 'en', teaching_task=True)
        return memory, answer

    def test_timeout_then_restart_keeps_fact_on_real_disk(self):
        first, answer = self.teach_with_timeout()
        self.assertIn('saved this information on this machine', answer)
        self.assertIn('pending Hindsight confirmation', answer)
        restarted = CorrectionMemory(storage_path=self.path)
        self.assertEqual(first.session, restarted.session)
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
        self.assertEqual(self.path.parent.stat().st_mode & 0o777, 0o700)
        with patch.object(restarted, 'recall', side_effect=MemoryUnavailable(category='connection')):
            model = client('answer', answer='The battery is blue, according to you.')
            response = restarted.respond(model, 'test', 'What color is the battery?', '', 'en')
        self.assertIn('blue', response)
        data = json.loads(model.chat.completions.create.call_args.kwargs['messages'][1]['content'])
        self.assertEqual(len(data['corrections']), 1)

    def test_retry_uses_same_id_then_removes_only_confirmed_pending_record(self):
        first, _ = self.teach_with_timeout()
        document_id = next(iter(first.session))
        restarted = CorrectionMemory(storage_path=self.path)
        with patch.object(restarted, 'recall', return_value=[]), patch.object(restarted, 'retain') as retain:
            restarted.respond(client('answer', answer='Blue'), 'test', 'color?', '', 'en')
        self.assertEqual(retain.call_args.args[1], document_id)
        self.assertEqual(PendingMemory(self.path, restarted._outbox.namespace).load(), {})
        self.assertIn(document_id, restarted.session)

    def test_failed_retry_is_durable_and_at_most_once_per_session(self):
        self.teach_with_timeout()
        restarted = CorrectionMemory(storage_path=self.path)
        with patch.object(restarted, 'recall', return_value=[]), patch.object(
                restarted, 'retain', side_effect=MemoryUnavailable(category='timeout')) as retain:
            for _ in range(2):
                restarted.respond(client('answer', answer='Blue'), 'test', 'color?', '', 'en')
        retain.assert_called_once()
        self.assertTrue(CorrectionMemory(storage_path=self.path).session)

    def test_disk_failure_never_claims_local_persistence(self):
        memory = CorrectionMemory(storage_path=self.path)
        with patch.object(memory._outbox, 'save', side_effect=OSError), patch.object(
                memory, 'recall', return_value=[]), patch.object(
                memory, 'retain', side_effect=MemoryUnavailable(category='timeout')):
            answer = memory.respond(client('correct', fact='Robot TEST uses blue battery'),
                                    'test', 'Teach', '', 'en', teaching_task=True)
        self.assertIn('in this conversation', answer)
        self.assertNotIn('saved this information on this machine', answer)

    def test_bank_isolation(self):
        self.teach_with_timeout()
        with patch.dict(os.environ, {'HINDSIGHT_BANK_ID': 'different-bank'}):
            self.assertEqual(CorrectionMemory(storage_path=self.path).session, {})

    def test_old_acknowledgement_cannot_delete_newer_edit(self):
        store = PendingMemory(self.path, 'bank')
        old = {'document_id': 'same', 'text': 'old', 'metadata': {}}
        new = {**old, 'text': 'new'}
        store.save(old)
        store.save(new)
        store.acknowledge(old)
        self.assertEqual(store.load()['same']['text'], 'new')

    def test_questions_do_not_create_pending_database(self):
        memory = CorrectionMemory(storage_path=self.path)
        with patch.object(memory, 'recall', return_value=[]):
            memory.respond(client('fallback'), 'test', 'Who is X?', '', 'en', allow_updates=False)
        self.assertFalse(self.path.exists())
