import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robot_ui.conversation_policy import conversation_messages, plan_turn, answer_turn
from robot_ui.correction_memory import CorrectionMemory


class ConversationPolicyTests(unittest.TestCase):
    def test_missing_university_total_falls_back_to_web(self):
        from robot_ui.iuh_local_search import IuhLocalSearch
        question = "Trường Đại học Công nghiệp có bao nhiêu khoa?"
        web = Mock(return_value="verified answer")
        local, memory = Mock(), Mock()
        answer = answer_turn(
            {"route": "local", "query": question}, question, "history",
            search_local=IuhLocalSearch().search, answer_local=local,
            answer_web=web, answer_general=Mock(), answer_memory=memory)
        self.assertEqual(answer, "verified answer")
        web.assert_called_once_with(question, "history")
        local.assert_not_called()
        memory.assert_not_called()

    def test_valid_plan_preserves_one_turn_language_and_session_default(self):
        client = Mock()
        expected = {"route": "general", "query": "Translate and explain",
                    "memory_task": False, "reply_language": "en",
                    "conversation_language": "vi"}
        client.chat.completions.create.return_value = SimpleNamespace(choices=[
            SimpleNamespace(message=SimpleNamespace(content=json.dumps(expected)))])
        result = plan_turn(client, "test", [{"role": "user", "content": "request"}],
                           state_context="current facts", default_language="vi")
        self.assertEqual(result, expected)
        payload = json.loads(client.chat.completions.create.call_args.kwargs["messages"][1]["content"])
        self.assertEqual(payload["default_language"], "vi")
        self.assertEqual(payload["user_memory_data"], "current facts")

    def test_mixed_update_and_question_composes_after_storage(self):
        events = []
        def remember(*args):
            events.append("remember")
            return "Updated K-481; long-term save not confirmed"
        def compose(instruction):
            events.append("answer")
            self.assertIn("long-term save not confirmed", instruction)
            return "K-481 replaced K-418; fictional data."
        result = answer_turn(
            {"route": "general", "query": "update and compare", "memory_task": True},
            "update and compare", "history",
            search_local=Mock(return_value=SimpleNamespace(status="insufficient")),
            answer_local=Mock(), answer_web=Mock(), answer_general=compose,
            answer_memory=remember)
        self.assertEqual(events, ["remember", "answer"])
        self.assertIn("K-418", result)

    def test_mixed_web_request_gets_memory_status_without_second_write(self):
        memory = Mock(return_value="saved user preference")
        web = Mock(return_value="web answer")
        answer_turn(
            {"route": "web", "query": "verify schedule", "memory_task": True},
            "remember and verify", "history",
            search_local=Mock(return_value=SimpleNamespace(status="insufficient")),
            answer_local=Mock(), answer_web=web, answer_general=Mock(), answer_memory=memory)
        memory.assert_called_once()
        self.assertIn("saved user preference", web.call_args.args[1])

    def test_history_keeps_more_than_eight_messages_and_excludes_database(self):
        history = [{"role": "system", "content": "IUH database"}]
        history += [{"role": "user" if i % 2 == 0 else "assistant",
                     "content": str(i)} for i in range(24)]
        self.assertEqual(conversation_messages(history), history[1:])

    def test_truncated_history_is_disclosed_and_latest_turn_is_preserved(self):
        history = [{"role": "user", "content": "old"},
                   {"role": "assistant", "content": "previous"},
                   {"role": "user", "content": "latest"}]
        result = conversation_messages(history, max_chars=6)
        self.assertEqual(result[0]["role"], "system")
        self.assertEqual(result[1:], history[-1:])

    def test_plan_validates_structured_output(self):
        client = Mock()
        for invalid in ({}, [], {"route": "execute", "query": "x"},
                        {"route": "general", "query": " "}):
            with self.subTest(invalid=invalid):
                client.chat.completions.create.return_value = SimpleNamespace(
                    choices=[SimpleNamespace(message=SimpleNamespace(content=json.dumps(invalid)))])
                with self.assertRaises(ValueError):
                    plan_turn(client, "test", [{"role": "user", "content": "hello"}])

    def test_retrieval_uses_resolved_query_but_answer_keeps_original_request(self):
        search = Mock(return_value=SimpleNamespace(status="sufficient", evidence=[]))
        local, web, general = Mock(return_value="answer"), Mock(), Mock()
        memory = Mock(return_value=None)
        answer = answer_turn(
            {"route": "local", "query": "Email Phòng Đào tạo IUH?"},
            "Email của phòng đó?", "context", search_local=search,
            answer_local=local, answer_web=web, answer_general=general,
            answer_memory=memory)
        self.assertEqual(answer, "answer")
        search.assert_called_once_with("Email Phòng Đào tạo IUH?")
        self.assertEqual(local.call_args.args[:2], ("Email của phòng đó?", "context"))
        web.assert_not_called()

    def test_memory_fallback_does_not_trigger_unrelated_web_search(self):
        web, general = Mock(), Mock(return_value="cannot recall")
        answer = answer_turn(
            {"route": "memory", "query": "remember my room?"}, "room?", "",
            search_local=Mock(), answer_local=Mock(), answer_web=web,
            answer_general=general, answer_memory=Mock(return_value=None))
        self.assertEqual(answer, "cannot recall")
        web.assert_not_called()

    def test_test_fact_cannot_override_repository_evidence(self):
        memory = CorrectionMemory()
        local = [{"path": "room", "value": "B-100"}]
        recalled = [{"metadata": {"json_path": "room", "base_json_value": '"B-100"',
                                   "provenance": "test"}, "text": "B-732"}]
        self.assertEqual(memory._filter_for_local_scope(recalled, local), [])

    def test_test_correction_retains_provenance_without_repository_scope(self):
        memory, client = CorrectionMemory(), Mock()
        memory.enabled = True
        client.chat.completions.create.return_value = SimpleNamespace(choices=[
            SimpleNamespace(message=SimpleNamespace(content=json.dumps({
                "action": "correct", "fact": "Cô Mây Xanh: B-732",
                "document_id": "", "answer": "", "provenance": "test"})))])
        with patch.object(memory, "recall", return_value=[]), \
                patch.object(memory, "retain") as retain:
            memory.respond(client, "test", "ghi nhớ dữ liệu giả lập", "", "vi",
                           local_evidence=[{"path": "room", "value": "B-100"}])
        saved = next(iter(memory.session.values()))
        self.assertEqual(saved["metadata"]["provenance"], "test")
        self.assertNotIn("json_path", saved["metadata"])
        self.assertIn("Dữ liệu kiểm thử", saved["text"])
        self.assertEqual(retain.call_args.args[2]["provenance"], "test")

    def test_calculated_update_keeps_previous_snapshot_and_other_subject(self):
        memory = CorrectionMemory()
        memory.enabled = False
        def update(fact, document_id="", derivation="stated"):
            client = Mock()
            client.chat.completions.create.return_value = SimpleNamespace(choices=[
                SimpleNamespace(message=SimpleNamespace(content=json.dumps({
                    "action": "correct", "fact": fact, "document_id": document_id,
                    "answer": "", "provenance": "test", "derivation": derivation})))])
            memory.respond(client, "test", "scenario update", "", "vi")
        update("A=40%; B=65%")
        document_id = next(iter(memory.session))
        update("A=70%; B=65%", document_id, "calculated")
        current = memory.session[document_id]
        self.assertIn("A=70%; B=65%", current["text"])
        self.assertEqual(current["metadata"]["derivation"], "calculated")
        revisions = json.loads(current["metadata"]["revisions_json"])
        self.assertIn("A=40%; B=65%", revisions[0]["text"])
        self.assertEqual(len(memory.session), 1)
        self.assertIn("A=70%", memory.session_context())

    def test_revisions_survive_persisted_record_and_failed_next_save(self):
        memory, client = CorrectionMemory(), Mock()
        memory.enabled = True
        prior = {"document_id": "code", "text": "K-481", "metadata": {
            "provenance": "test", "revisions_json": json.dumps([{"text": "K-418"}])}}
        client.chat.completions.create.return_value = SimpleNamespace(choices=[
            SimpleNamespace(message=SimpleNamespace(content=json.dumps({
                "action": "correct", "fact": "Dữ liệu kiểm thử: Dữ liệu giả lập: K-999",
                "document_id": "code", "answer": "", "provenance": "test"})))])
        from robot_ui.correction_memory import MemoryUnavailable
        with patch.object(memory, "recall", return_value=[prior]), \
                patch.object(memory, "retain", side_effect=MemoryUnavailable) as retain:
            answer = memory.respond(client, "test", "update code", "", "vi")
        saved = memory.session["code"]
        self.assertEqual(saved["text"], "Dữ liệu kiểm thử: K-999")
        revisions = json.loads(saved["metadata"]["revisions_json"])
        self.assertEqual([r["text"] for r in revisions], ["K-418", "K-481"])
        self.assertEqual(retain.call_args.args[2]["revisions_json"],
                         saved["metadata"]["revisions_json"])
        self.assertIn("chưa xác nhận", answer)

    def test_unknown_relative_update_does_not_write(self):
        memory, client = CorrectionMemory(), Mock()
        memory.enabled = False
        client.chat.completions.create.return_value = SimpleNamespace(choices=[
            SimpleNamespace(message=SimpleNamespace(content=json.dumps({
                "action": "clarify", "fact": "", "document_id": "",
                "answer": "Giá trị ban đầu là bao nhiêu?"})))])
        with patch.object(memory, "retain") as retain:
            result = memory.respond(client, "test", "A tăng thêm 30", "", "vi")
        self.assertEqual(memory.session, {})
        retain.assert_not_called()
        self.assertIn("ban đầu", result)


if __name__ == "__main__":
    unittest.main()
