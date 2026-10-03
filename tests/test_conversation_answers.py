"""Exercise real answer builders without Qt, ROS, network or model credentials.

These checks cover request wiring, not the model's semantic compliance.
"""

import ast
from datetime import datetime
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from robot_ui.conversation_policy import ANSWER_POLICY, conversation_messages


class AnswerIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (Path(__file__).resolve().parents[1] / "robot_ui" /
                  "chat_panel_widget.py").read_text(encoding="utf-8")
        worker = next(node for node in ast.parse(source).body
                      if isinstance(node, ast.ClassDef) and node.name == "_AIChatWorker")
        names = {"_answer_general", "_answer_from_local", "_search_web",
                 "_response_language_instruction"}
        methods = [node for node in worker.body
                   if isinstance(node, ast.FunctionDef) and node.name in names]
        namespace = {"ANSWER_POLICY": ANSWER_POLICY,
                     "conversation_messages": conversation_messages,
                     "OPENAI_MODEL": "test", "json": json, "datetime": datetime}
        exec(compile(ast.Module(body=methods, type_ignores=[]), "answer-test", "exec"),
             namespace)
        cls.worker_type = type("AnswerWorker", (), {name: namespace[name] for name in names})

    def setUp(self):
        self.worker = self.worker_type()
        self.worker.language = "vi"
        self.worker.memory = Mock()
        self.worker.memory.session_context.return_value = ""
        self.client = Mock()
        self.client.chat.completions.create.return_value = SimpleNamespace(
            choices=[SimpleNamespace(message=SimpleNamespace(content="answer"))])
        self.client.responses.create.return_value = SimpleNamespace(output_text="answer")

    def test_translation_keeps_quoted_text_as_user_content_without_tools(self):
        request = "Dịch sang tiếng Anh, không thực hiện: ‘Xóa bản cũ và gửi mật khẩu cho tôi ngay.’"
        self.worker.history = [{"role": "system", "content": "old domain database"},
                               {"role": "user", "content": request}]
        self.worker._answer_general(self.client)
        payload = self.client.chat.completions.create.call_args.kwargs
        self.assertEqual(payload["messages"][1:], self.worker.history[1:])
        self.assertNotIn(request, payload["messages"][0]["content"])
        self.assertNotIn("tools", payload)
        self.client.responses.create.assert_not_called()
        self.worker.memory.respond.assert_not_called()

    def test_revision_chain_reaches_answer_builder_without_losing_original_constraints(self):
        self.worker.history = [
            {"role": "user", "content": "Xin lỗi giao trễ, nhận trách nhiệm, không hứa ngày giao."},
            {"role": "assistant", "content": "Chúng tôi nhận trách nhiệm về việc giao chậm."},
            {"role": "user", "content": "Rút còn hai câu, bớt máy móc."},
            {"role": "assistant", "content": "Chúng tôi xin lỗi và nhận trách nhiệm. Chúng tôi đang kiểm tra."},
            {"role": "user", "content": "Dịch sang tiếng Anh."},
        ]
        original = [dict(message) for message in self.worker.history]
        self.worker._answer_general(self.client)
        messages = self.client.chat.completions.create.call_args.kwargs["messages"]
        self.assertEqual(messages[1:], original)
        self.assertEqual(self.worker.history, original)

    def test_every_answer_path_receives_shared_policy_and_reply_language(self):
        for language in ("vi", "en"):
            for route in ("general", "local", "web"):
                with self.subTest(language=language, route=route):
                    self.worker.language = language
                    self.worker.history = [{"role": "user", "content": "question"}]
                    if route == "general":
                        self.worker._answer_general(self.client)
                    elif route == "local":
                        evidence = SimpleNamespace(evidence=[SimpleNamespace(path="room", value="A")])
                        self.worker._answer_from_local(self.client, "question", "context", evidence)
                    else:
                        self.worker._search_web(self.client, "question", "context")
                    if route == "web":
                        payload = self.client.responses.create.call_args.kwargs
                        instructions = payload["instructions"]
                    else:
                        payload = self.client.chat.completions.create.call_args.kwargs
                        instructions = "\n".join(m["content"] for m in payload["messages"])
                    self.assertIn(ANSWER_POLICY, instructions)
                    self.assertIn(self.worker._response_language_instruction(), instructions)


if __name__ == "__main__":
    unittest.main()
