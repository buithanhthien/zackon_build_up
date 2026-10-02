import ast
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.error import URLError

from robot_ui.correction_memory import CorrectionMemory, MemoryUnavailable


def client_for(action, fact="", document_id="", answer=""):
    client = Mock()
    content = json.dumps(dict(action=action, fact=fact, document_id=document_id, answer=answer))
    client.chat.completions.create.return_value = SimpleNamespace(
        choices=[SimpleNamespace(message=SimpleNamespace(content=content))]
    )
    return client


class MemoryTests(unittest.TestCase):
    def setUp(self):
        env = patch.dict(os.environ, {}, clear=True)
        env.start()
        self.addCleanup(env.stop)
        self.memory = CorrectionMemory()

    def respond(self, client):
        return self.memory.respond(client, "test-model", "Where is A?", "A was at X5.4", "en")

    def test_disabled_skips_storage_but_still_classifies(self):
        self.memory.enabled = False
        client = client_for("fallback")
        with patch.object(self.memory, "recall") as recall:
            self.assertIsNone(self.respond(client))
        recall.assert_not_called()
        client.chat.completions.create.assert_called_once()

    def test_empty_bank_falls_through_without_retaining_web_answers(self):
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain") as retain:
            self.assertIsNone(self.respond(client_for("fallback")))
            retain.assert_not_called()

    def test_explicit_correction_saved_before_acknowledgement(self):
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain") as retain:
            answer = self.respond(client_for("correct", fact="A is at X5.7"))
        self.assertIn("saved", answer)
        self.assertEqual(retain.call_args.args[0], "A is at X5.7")
        self.assertTrue(retain.call_args.args[1].startswith("correction-"))

    def test_second_correction_reuses_document(self):
        with patch.object(self.memory, "recall", return_value=[{"document_id": "a-room"}]), \
                patch.object(self.memory, "retain") as retain:
            self.respond(client_for("correct", fact="A is at X5.8", document_id="a-room"))
        retain.assert_called_once_with("A is at X5.8", "a-room")

    def test_unknown_document_cannot_be_overwritten(self):
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain") as retain:
            with self.assertRaises(ValueError):
                self.respond(client_for("correct", fact="A is at X5.8", document_id="foreign"))
            retain.assert_not_called()

    def test_ambiguous_correction_asks_without_writing(self):
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain") as retain:
            self.assertEqual(self.respond(client_for("clarify", answer="Whose room?")), "Whose room?")
            retain.assert_not_called()

    def test_retain_failure_does_not_claim_success(self):
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain", side_effect=MemoryUnavailable):
            answer = self.respond(client_for("correct", fact="A is at X5.7"))
        self.assertIn("not confirmed a long-term save", answer)
        self.assertEqual(next(iter(self.memory.session.values()))["text"], "A is at X5.7")

    def test_recall_failure_allows_fallback_with_warning(self):
        client = client_for("fallback")
        with patch.object(self.memory, "recall", side_effect=MemoryUnavailable):
            self.assertIsNone(self.respond(client))
        self.assertIn("unavailable", self.memory.warning)
        client.chat.completions.create.assert_called_once()

    def test_disabled_negative_correction_survives_later_turns(self):
        self.memory.enabled = False
        fact = "Khoa Công nghệ Điện không có ngành Điện tử - Viễn thông."
        with patch.object(self.memory, "retain") as retain:
            answer = self.memory.respond(client_for("correct", fact=fact), "test",
                                         fact, "Robot previously claimed it does.", "vi")
        self.assertIn(fact, answer)
        self.assertIn("chưa xác nhận", answer)
        retain.assert_not_called()
        later = client_for("answer", answer=fact)
        self.assertEqual(self.memory.respond(later, "test", "Có Viễn thông không?", "", "vi"), fact)
        payload = json.loads(later.chat.completions.create.call_args.kwargs["messages"][1]["content"])
        self.assertEqual(payload["corrections"][0]["text"], fact)

    def test_session_update_replaces_stale_persistent_record(self):
        self.memory.session["a-room"] = {"text": "A is at X5.8", "document_id": "a-room"}
        client = client_for("answer", answer="X5.8")
        with patch.object(self.memory, "recall", return_value=[{
            "text": "A is at X5.7", "document_id": "a-room"
        }]):
            self.assertEqual(self.respond(client), "X5.8")
        payload = json.loads(client.chat.completions.create.call_args.kwargs["messages"][1]["content"])
        self.assertEqual([m["text"] for m in payload["corrections"]], ["A is at X5.8"])

    def test_offline_correction_is_kept_in_session(self):
        with patch.object(self.memory, "recall", side_effect=MemoryUnavailable), \
                patch.object(self.memory, "retain") as retain:
            answer = self.respond(client_for("correct", fact="A is at X5.7"))
        self.assertIn("this conversation", answer)
        self.assertEqual(len(self.memory.session), 1)
        retain.assert_not_called()

    def test_new_client_reads_persisted_memory(self):
        self.memory = CorrectionMemory()
        with patch.object(self.memory, "_request", side_effect=[{}, {"results": [
            {"text": "A is at X5.7", "document_id": "a-room"}
        ]}]):
            self.assertEqual(self.respond(client_for("answer", answer="X5.7")), "X5.7")

    def test_http_body_and_synchronous_success(self):
        payload = io.BytesIO(b'{"success": true, "async": false}')
        with patch("robot_ui.correction_memory.urlopen", return_value=payload) as open_url:
            self.memory.retain("A is at X5.7", "a-room")
        request = open_url.call_args.args[0]
        data = json.loads(request.data)
        self.assertEqual(request.method, "POST")
        self.assertTrue(request.full_url.endswith("/beson-corrections/memories"))
        self.assertIs(data["async"], False)
        self.assertEqual(data["items"][0]["document_id"], "a-room")
        self.assertEqual(data["items"][0]["metadata"]["source"], "user_correction")

    def test_queued_or_failed_save_is_not_confirmed(self):
        for response in ({"success": True, "async": True}, {"success": False, "async": False}):
            with patch.object(self.memory, "_request", return_value=response):
                with self.assertRaises(MemoryUnavailable):
                    self.memory.retain("fact", "doc")

    def test_http_errors_do_not_leak_response(self):
        with patch("robot_ui.correction_memory.urlopen", side_effect=URLError("private data")):
            with self.assertRaisesRegex(MemoryUnavailable, "^Hindsight request failed$"):
                self.memory.recall("question")

    def test_custom_compose_port_and_bank(self):
        with patch.dict(os.environ, {"HINDSIGHT_API_PORT": "8890", "HINDSIGHT_BANK_ID": "a/b"}):
            self.assertEqual(CorrectionMemory().url, "http://127.0.0.1:8890/v1/default/banks/a%2Fb")


class WorkerIntegrationTests(unittest.TestCase):
    def test_memory_precedes_local_and_web_and_always_finishes(self):
        # Execute the actual worker method without importing Qt/ROS/audio hardware.
        source = Path("robot_ui/chat_panel_widget.py").read_text()
        tree = ast.parse(source)
        worker = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "_AIChatWorker")
        run = next(n for n in worker.body if isinstance(n, ast.FunctionDef) and n.name == "run")
        module = ast.Module(body=[run], type_ignores=[])
        for memory_answer in ("X5.7", None):
            factory = Mock()
            factory.return_value.respond.return_value = memory_answer
            namespace = {"OpenAI": Mock(), "OPENAI_API_KEY": "test", "OPENAI_MODEL": "test",
                         "CorrectionMemory": factory}
            exec(compile(module, "worker-test", "exec"), namespace)
            instance = Mock(language="en")
            instance.memory = factory.return_value
            instance.memory.warning = ""
            instance._get_latest_user_question.return_value = "Where is A?"
            instance._get_recent_context.return_value = "context"
            instance._check_database.return_value = {"source": "web"}
            instance._search_web.return_value = "web answer"
            namespace["run"](instance)
            if memory_answer:
                instance._check_database.assert_not_called()
                instance._search_web.assert_not_called()
            else:
                instance._check_database.assert_called_once()
                instance._search_web.assert_called_once()
            instance.response_ready.emit.assert_called_once_with(memory_answer or "web answer")
            instance.finished.emit.assert_called_once()
            instance.error_occurred.emit.assert_not_called()


if __name__ == "__main__":
    unittest.main()
