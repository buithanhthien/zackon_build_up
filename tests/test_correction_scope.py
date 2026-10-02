import json
import os
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robot_ui.correction_memory import CorrectionMemory, MemoryUnavailable


def client_for(action, fact="", document_id="", answer=""):
    client = Mock()
    content = json.dumps({
        "action": action,
        "fact": fact,
        "document_id": document_id,
        "answer": answer,
    })
    client.chat.completions.create.return_value = SimpleNamespace(
        choices=[SimpleNamespace(message=SimpleNamespace(content=content))]
    )
    return client


def evidence(path, value):
    return [SimpleNamespace(path=path, value=value)]


class CorrectionScopeTests(unittest.TestCase):
    def setUp(self):
        env = patch.dict(os.environ, {}, clear=True)
        env.start()
        self.addCleanup(env.stop)
        self.memory = CorrectionMemory()

    def test_scoped_correction_stores_json_path_and_base_value(self):
        local = evidence("lien_he.phong_dao_tao", "old@iuh.edu.vn")
        client = client_for("correct", fact="Email phòng đào tạo là new@iuh.edu.vn")
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain") as retain:
            self.memory.respond(client, "test", "Email mới là new@iuh.edu.vn", "", "vi", local)

        saved = next(iter(self.memory.session.values()))
        self.assertEqual(saved["metadata"]["json_path"], "lien_he.phong_dao_tao")
        self.assertEqual(saved["metadata"]["base_json_value"], '"old@iuh.edu.vn"')
        retain.assert_called_once()
        self.assertEqual(retain.call_args.args[2]["json_path"], "lien_he.phong_dao_tao")

    def test_unrelated_scoped_correction_is_not_shown_to_model(self):
        self.memory.session["faculty-email"] = {
            "text": "Email khoa là feet-new@iuh.edu.vn",
            "document_id": "faculty-email",
            "metadata": {
                "source": "user_correction",
                "json_path": "khoa_cong_nghe_dien.lien_he_truc_tiep.email",
                "base_json_value": '"feet@iuh.edu.vn"',
            },
        }
        local = evidence("lien_he.phong_dao_tao", "phongdaotao@iuh.edu.vn")
        client = client_for("fallback")
        with patch.object(self.memory, "recall", return_value=[]):
            self.assertIsNone(self.memory.respond(client, "test", "Email phòng đào tạo?", "", "vi", local))
        payload = json.loads(client.chat.completions.create.call_args.kwargs["messages"][1]["content"])
        self.assertEqual(payload["corrections"], [])

    def test_matching_scoped_correction_is_shown_to_model(self):
        self.memory.session["training-email"] = {
            "text": "Email phòng đào tạo là new@iuh.edu.vn",
            "document_id": "training-email",
            "metadata": {
                "source": "user_correction",
                "json_path": "lien_he.phong_dao_tao",
                "base_json_value": '"phongdaotao@iuh.edu.vn"',
            },
        }
        local = evidence("lien_he.phong_dao_tao", "phongdaotao@iuh.edu.vn")
        client = client_for("answer", answer="new@iuh.edu.vn")
        with patch.object(self.memory, "recall", return_value=[]):
            answer = self.memory.respond(client, "test", "Email phòng đào tạo?", "", "vi", local)
        self.assertEqual(answer, "new@iuh.edu.vn")
        payload = json.loads(client.chat.completions.create.call_args.kwargs["messages"][1]["content"])
        self.assertEqual([m["document_id"] for m in payload["corrections"]], ["training-email"])

    def test_correction_becomes_stale_when_repository_value_changes(self):
        self.memory.session["training-email"] = {
            "text": "Email phòng đào tạo là new@iuh.edu.vn",
            "document_id": "training-email",
            "metadata": {
                "source": "user_correction",
                "json_path": "lien_he.phong_dao_tao",
                "base_json_value": '"old@iuh.edu.vn"',
            },
        }
        local = evidence("lien_he.phong_dao_tao", "repository-newer@iuh.edu.vn")
        client = client_for("fallback")
        with patch.object(self.memory, "recall", return_value=[]):
            self.assertIsNone(self.memory.respond(client, "test", "Email phòng đào tạo?", "", "vi", local))
        payload = json.loads(client.chat.completions.create.call_args.kwargs["messages"][1]["content"])
        self.assertEqual(payload["corrections"], [])

    def test_hindsight_down_still_keeps_scoped_correction_in_session(self):
        local = evidence("lien_he.phong_dao_tao", "old@iuh.edu.vn")
        client = client_for("correct", fact="Email phòng đào tạo là new@iuh.edu.vn")
        with patch.object(self.memory, "recall", side_effect=MemoryUnavailable), \
                patch.object(self.memory, "retain") as retain:
            answer = self.memory.respond(client, "test", "Email mới là new@iuh.edu.vn", "", "vi", local)
        self.assertIn("chưa xác nhận", answer)
        saved = next(iter(self.memory.session.values()))
        self.assertEqual(saved["metadata"]["json_path"], "lien_he.phong_dao_tao")
        retain.assert_not_called()

    def test_same_json_path_uses_stable_document_id_across_base_changes(self):
        path = "lien_he.phong_dao_tao"
        first_local = evidence(path, "old@iuh.edu.vn")
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain"):
            self.memory.respond(
                client_for("correct", fact="Email là first@iuh.edu.vn"),
                "test", "sửa email", "", "vi", first_local,
            )
        first_id = next(iter(self.memory.session))

        second_local = evidence(path, "repository-new@iuh.edu.vn")
        with patch.object(self.memory, "recall", return_value=[]), \
                patch.object(self.memory, "retain"):
            self.memory.respond(
                client_for("correct", fact="Email là second@iuh.edu.vn"),
                "test", "sửa lại email", "", "vi", second_local,
            )
        self.assertEqual(list(self.memory.session), [first_id])
        self.assertEqual(
            self.memory.session[first_id]["metadata"]["base_json_value"],
            '"repository-new@iuh.edu.vn"',
        )


if __name__ == "__main__":
    unittest.main()
