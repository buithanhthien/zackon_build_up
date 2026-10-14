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
from robot_ui.conversation_policy import answer_turn


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
        self.memory = CorrectionMemory(storage_path=False)

    def respond(self, client):
        return self.memory.respond(client, "test-model", "Where is A?", "A was at X5.4", "en")

    def test_teaching_empty_bank_extracts_and_saves_with_restricted_schema(self):
        client = client_for("correct", fact="A created the robot")
        with patch.object(self.memory, "recall", return_value=[]), patch.object(self.memory, "retain") as retain:
            answer = self.memory.respond(client, "test", "A created the robot", "", "en", teaching_task=True)
        retain.assert_called_once()
        schema = client.chat.completions.create.call_args.kwargs["response_format"]["json_schema"]["schema"]
        self.assertEqual(schema["properties"]["action"]["enum"], ["correct", "clarify"])
        self.assertIn("saved", answer)

    def test_invalid_teaching_fallback_cannot_silently_acknowledge(self):
        with patch.object(self.memory, "recall", return_value=[]), patch.object(self.memory, "retain") as retain:
            answer = self.memory.respond(client_for("fallback"), "test", "A created robot", "", "en", teaching_task=True)
        retain.assert_not_called()
        self.assertIn("not saved", answer)

    def test_teaching_clarification_does_not_write(self):
        with patch.object(self.memory, "recall", return_value=[]), patch.object(self.memory, "retain") as retain:
            answer = self.memory.respond(client_for("clarify", answer="Which fact should I save?"),
                                         "test", "wrong", "", "en", teaching_task=True)
        retain.assert_not_called()
        self.assertEqual(answer, "Which fact should I save?")

    def test_request_errors_have_safe_categories(self):
        from urllib.error import HTTPError
        cases = [(HTTPError("http://private", 401, "secret", {}, None), "http-auth"),
                 (HTTPError("http://private", 500, "secret", {}, None), "http-server"),
                 (URLError(TimeoutError("secret")), "timeout"),
                 (URLError("secret"), "connection")]
        for error, category in cases:
            with self.subTest(category=category), patch("robot_ui.correction_memory.urlopen", side_effect=error):
                with self.assertRaises(MemoryUnavailable) as caught:
                    self.memory._request("GET", "")
                self.assertEqual(caught.exception.category, category)
                self.assertNotIn("secret", str(caught.exception))

    def test_startup_is_ready_before_recall(self):
        events = []
        self.memory.startup_wait = lambda: events.append("ready") or True
        with patch.object(self.memory, "recall", side_effect=lambda q: events.append("recall") or []):
            self.respond(client_for("fallback"))
        self.assertEqual(events, ["ready", "recall"])

    def test_failed_startup_does_not_attempt_recall_or_claim_save(self):
        self.memory.startup_wait = lambda: False
        with patch.object(self.memory, "recall") as recall, patch.object(self.memory, "retain") as retain:
            answer = self.respond(client_for("correct", fact="A creates the robot"))
        recall.assert_not_called()
        retain.assert_not_called()
        self.assertIn("not confirmed", answer)

    def test_web_probe_cannot_write_model_invented_correction(self):
        with patch.object(self.memory, "recall", return_value=[]), patch.object(self.memory, "retain") as retain:
            result = self.memory.respond(client_for("correct", fact="invented"), "test",
                                         "Who is A?", "", "en", allow_updates=False)
        self.assertIsNone(result)
        self.assertEqual(self.memory.session, {})
        retain.assert_not_called()

    def test_restart_identity_question_uses_persisted_fact_before_web(self):
        bank = []
        fact = "Bùi Thanh Thiên là người tạo ra Bé Son, sinh viên Khoa Công nghệ Điện IUH."
        def retain(text, document_id, metadata):
            bank.append({"text": text, "document_id": document_id, "metadata": metadata})
        with patch.object(self.memory, "recall", return_value=[]), patch.object(self.memory, "retain", side_effect=retain):
            saved = self.memory.respond(client_for("correct", fact=fact), "test", fact, "", "vi")
        self.assertIn("đã lưu", saved)
        restarted = CorrectionMemory(storage_path=False)
        self.assertEqual(restarted.session, {})
        web = Mock()
        answer = "Theo thông tin bạn cung cấp, " + fact
        with patch.object(restarted, "recall", return_value=bank):
            result = answer_turn(
                {"route": "web", "query": "Bùi Thanh Thiên là ai?", "external_required": False},
                "Bùi Thanh Thiên là ai?", "", search_local=Mock(), answer_local=Mock(),
                answer_web=web, answer_general=Mock(),
                answer_memory=lambda q, c, e: restarted.respond(
                    client_for("answer", answer=answer), "test", q, c, "vi", allow_updates=False))
        self.assertEqual(result, answer)
        web.assert_not_called()

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
        retain.assert_called_once()
        self.assertEqual(retain.call_args.args[:2], ("A is at X5.8", "a-room"))
        self.assertEqual(retain.call_args.args[2]["derivation"], "stated")

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
        self.memory = CorrectionMemory(storage_path=False)
        with patch.object(self.memory, "_request", side_effect=[{}, {"results": [
            {"text": "A is at X5.7", "document_id": "a-room"}
        ]}, {"items": [{"id": "a-room", "retain_params": {"metadata": {"source": "user_correction"}}}]}]):
            self.assertEqual(self.respond(client_for("answer", answer="X5.7")), "X5.7")

    def test_long_history_is_not_sent_to_recall(self):
        client = client_for("fallback")
        history = "Earlier unrelated conversation. " * 2000
        with patch.object(self.memory, "recall", return_value=[]) as recall:
            self.memory.respond(client, "test", "Her room?", history, "en",
                                recall_query="Where is teacher A's room?")
        recall.assert_called_once_with("Where is teacher A's room?")
        payload = json.loads(client.chat.completions.create.call_args.kwargs["messages"][1]["content"])
        self.assertEqual(payload["history"], history)

    def test_recall_without_resolved_query_uses_question_only(self):
        with patch.object(self.memory, "recall", return_value=[]) as recall:
            self.respond(client_for("fallback"))
        recall.assert_called_once_with("Where is A?")

    def test_v049_metadata_restores_scoped_memory_in_new_session(self):
        metadata = {"source": "user_correction", "json_path": "room",
                    "base_json_value": '"X5.4"', "provenance": "user"}
        for params in ({"metadata": metadata}, json.dumps({"metadata": metadata})):
            with self.subTest(params=params):
                memory = CorrectionMemory(storage_path=False)
                responses = [{}, {"results": [
                    {"id": "unit-1", "text": "A is at X5.7", "document_id": "a-room", "metadata": None},
                    {"id": "unit-2", "text": "A teaches math", "document_id": "a-room", "metadata": None},
                ]}, {"items": [
                    {"id": "a-room-other", "retain_params": {"metadata": {"provenance": "test"}}},
                    {"id": "a-room", "retain_params": params},
                ], "total": 2}]
                client = client_for("answer", answer="X5.7")
                with patch.object(memory, "_request", side_effect=responses) as request:
                    answer = memory.respond(client, "test", "A's room?", "", "en",
                                            local_evidence=[{"path": "room", "value": "X5.4"}])
                self.assertEqual(answer, "X5.7")
                self.assertEqual(request.call_count, 3)
                payload = json.loads(client.chat.completions.create.call_args.kwargs["messages"][1]["content"])
                self.assertEqual(len(payload["corrections"]), 2)
                self.assertEqual(payload["corrections"][0]["metadata"], metadata)

    def test_metadata_lookup_requires_exact_document_and_handles_pagination(self):
        with patch.object(self.memory, "_request", side_effect=[
            {"items": [{"id": "doc-other"}], "total": 2},
            {"items": [{"id": "doc", "retain_params": {"metadata": {"provenance": "test"}}}], "total": 2},
        ]) as request:
            self.assertEqual(self.memory._document_metadata("doc"), {"provenance": "test"})
        self.assertIn("offset=1", request.call_args.args[1])

    def test_metadata_lookup_failure_does_not_use_unverified_scope(self):
        with patch.object(self.memory, "_request", side_effect=[{}, {"results": [
            {"document_id": "doc", "text": "room"}
        ]}, MemoryUnavailable()]):
            self.assertIsNone(self.respond(client_for("fallback")))
        self.assertIn("unavailable", self.memory.warning)

    def test_get_request_has_no_json_body(self):
        with patch("robot_ui.correction_memory.urlopen", return_value=io.BytesIO(b'{"items": []}')) as open_url:
            self.memory._request("GET", "/documents?q=doc")
        self.assertIsNone(open_url.call_args.args[0].data)

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
            self.assertEqual(CorrectionMemory(storage_path=False).url, "http://127.0.0.1:8890/v1/default/banks/a%2Fb")


class WorkerIntegrationTests(unittest.TestCase):
    def _run_worker(self, local_status="sufficient", memory_answer=None,
                    force_web=False, warning="", route=None, language_plan=None,
                    question="Where is A?", web_answer="web answer"):
        from robot_ui.web_sources import format_answer_sources
        # Execute the actual worker method without importing Qt/ROS/audio hardware.
        source = Path("robot_ui/chat_panel_widget.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        worker = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "_AIChatWorker")
        run = next(n for n in worker.body if isinstance(n, ast.FunctionDef) and n.name == "run")
        module = ast.Module(body=[run], type_ignores=[])
        namespace = {
            "OpenAI": Mock(),
            "OPENAI_API_KEY": "test",
            "OPENAI_MODEL": "test",
            "plan_turn": Mock(return_value={
                "route": route or ("web" if force_web else "local"),
                "query": "Where is A?",
                **(language_plan or {}),
            }),
            "answer_turn": answer_turn,
            "format_answer_sources": format_answer_sources,
        }
        exec(compile(module, "worker-test", "exec"), namespace)

        evidence = (SimpleNamespace(path="lien_he.phong_dao_tao", value="x"),)
        local_result = SimpleNamespace(
            status=local_status,
            evidence=evidence if local_status != "insufficient" else (),
            reason="test",
        )
        instance = Mock(language="en")
        instance.memory.warning = warning
        instance.memory.respond.return_value = memory_answer
        instance._get_latest_user_question.return_value = question
        instance._get_recent_context.return_value = "context"
        instance._search_local.return_value = local_result
        instance._answer_from_local.return_value = "local answer"
        instance._search_web.return_value = web_answer
        instance._clarify_local.return_value = "clarify"
        instance._answer_general.return_value = "general answer"

        namespace["run"](instance)
        return instance

    def test_teaching_intent_is_forwarded_to_extractor(self):
        instance = self._run_worker(route="memory", memory_answer="saved",
                                    language_plan={"memory_task": True})
        self.assertTrue(instance.memory.respond.call_args.kwargs["teaching_task"])

    def test_memory_route_reports_outage_in_final_reply(self):
        instance = self._run_worker(route="memory", warning="Cannot read long-term memory")
        self.assertIn("Cannot read long-term memory", instance.response_ready.emit.call_args.args[0])

    def test_local_miss_reports_memory_outage_before_web(self):
        instance = self._run_worker(local_status="insufficient", warning="Cannot read memory",
                                    language_plan={"external_required": False})
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("Cannot read memory")

    def test_web_identity_probe_answers_from_memory(self):
        instance = self._run_worker(force_web=True, memory_answer="user-taught identity",
                                    language_plan={"external_required": False})
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("user-taught identity")

    def test_identity_probe_outage_does_not_silently_search_web(self):
        instance = self._run_worker(force_web=True, warning="memory unavailable",
                                    language_plan={"external_required": False})
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("memory unavailable")

    def test_explicit_verification_gets_identity_context_but_still_searches(self):
        instance = self._run_worker(force_web=True, memory_answer="robot creator at IUH",
                                    language_plan={"external_required": True})
        instance._search_web.assert_called_once()
        self.assertIn("robot creator at IUH", instance._search_web.call_args.args[2])

    def test_web_citations_are_filtered_before_response_signal(self):
        answer = "Thầy là giảng viên. ([feet.iuh.edu.vn](https://feet.iuh.edu.vn/))"
        instance = self._run_worker(force_web=True, question="Giới thiệu thầy", web_answer=answer)
        instance.response_ready.emit.assert_called_once_with("Thầy là giảng viên.")
        instance.error_occurred.emit.assert_not_called()
        requested = self._run_worker(force_web=True, question="Cho tôi nguồn", web_answer=answer)
        requested.response_ready.emit.assert_called_once_with(answer)

    def test_local_evidence_answers_without_web(self):
        instance = self._run_worker(local_status="sufficient")
        instance._answer_from_local.assert_called_once()
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("local answer")

    def test_memory_can_override_matching_local_evidence(self):
        instance = self._run_worker(local_status="sufficient", memory_answer="corrected")
        instance._answer_from_local.assert_not_called()
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("corrected")

    def test_insufficient_local_evidence_falls_back_to_web(self):
        instance = self._run_worker(local_status="insufficient")
        instance._search_web.assert_called_once()
        instance.response_ready.emit.assert_called_once_with("web answer")

    def test_current_question_forces_web_and_disables_old_memory_answer(self):
        instance = self._run_worker(local_status="sufficient", force_web=True)
        instance._search_web.assert_called_once()
        instance.memory.respond.assert_called_once()
        self.assertFalse(instance.memory.respond.call_args.kwargs["allow_updates"])

    def test_ambiguous_retrieval_does_not_force_clarification(self):
        instance = self._run_worker(local_status="ambiguous")
        instance.memory.respond.assert_called_once()
        instance._search_web.assert_called_once()
        instance.response_ready.emit.assert_called_once_with("web answer")

    def test_missing_local_fact_can_be_answered_from_persisted_memory(self):
        instance = self._run_worker(local_status="insufficient", memory_answer="X5.7")
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("X5.7")
        self.assertIsNone(instance.memory.respond.call_args.kwargs["local_evidence"])
        self.assertEqual(instance.memory.respond.call_args.kwargs["recall_query"], "Where is A?")

    def test_memory_warning_does_not_block_local_answer(self):
        instance = self._run_worker(
            local_status="sufficient",
            warning="memory offline: ",
        )
        instance._answer_from_local.assert_called_once()
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("local answer")

    def test_general_task_skips_iuh_retrieval_and_memory(self):
        instance = self._run_worker(route="general", warning="memory offline: ")
        instance._search_local.assert_not_called()
        instance.memory.respond.assert_not_called()
        instance._search_web.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("general answer")
        instance.finished.emit.assert_called_once()

    def test_reply_language_and_future_preference_are_forwarded_separately(self):
        instance = self._run_worker(route="memory", language_plan={
            "reply_language": "en", "conversation_language": "vi"})
        instance.language_ready.emit.assert_called_once_with("en", "vi")
        self.assertEqual(instance.memory.respond.call_args.args[4], "en")

    def test_clarification_uses_conversation_instead_of_iuh_template(self):
        instance = self._run_worker(route="clarify")
        instance._search_local.assert_not_called()
        instance._clarify_local.assert_not_called()
        instance._answer_general.assert_called_once()

    def test_explicit_memory_turn_preserves_save_acknowledgement(self):
        instance = self._run_worker(route="memory", memory_answer="not saved long term")
        instance._search_local.assert_called_once()
        self.assertTrue(instance.memory.respond.call_args.kwargs["local_evidence"])
        instance._answer_general.assert_not_called()
        instance.response_ready.emit.assert_called_once_with("not saved long term")

    def test_explicit_web_request_forces_web_even_if_local_matches(self):
        instance = self._run_worker(
            local_status="sufficient",
            force_web=True,
        )

        instance._search_web.assert_called_once()
        instance._answer_from_local.assert_not_called()
        instance.response_ready.emit.assert_called_once_with(
            "web answer"
        )
    


if __name__ == "__main__":
    unittest.main()
