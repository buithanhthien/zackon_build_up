import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from robot_ui import codex_router


class CodexRouterTests(unittest.TestCase):
    def setUp(self):
        self.client = Mock()
        self.client.chat.completions.create.return_value = SimpleNamespace(
            choices=[SimpleNamespace(message=SimpleNamespace(
                content=json.dumps({"action": "none"})
            ))]
        )

    def set_classifier_response(self, data):
        self.client.chat.completions.create.return_value = SimpleNamespace(
            choices=[SimpleNamespace(message=SimpleNamespace(
                content=json.dumps(data, ensure_ascii=False)
            ))]
        )

    def test_ordinary_chat_does_not_call_action_classifier(self):
        result = codex_router.classify_codex_task(
            self.client, "test-model", "IUH có bao nhiêu khoa?"
        )
        self.assertIsNone(result)
        self.client.chat.completions.create.assert_not_called()

    def test_direct_desktop_request_hands_original_text_to_codex(self):
        request = 'Mở YouTube nghe bài "đếch cần gì nhiều ngoài em"'
        self.set_classifier_response({"action": "codex_task"})
        result = codex_router.classify_codex_task(
            self.client, "test-model", request
        )
        self.assertEqual(result, {
            "action": "codex_task",
            "task": request,
        })

    def test_absolute_project_folder_path_opens_without_calling_classifier(self):
        folder = codex_router.SOURCE_PATH
        result = codex_router.classify_codex_task(
            self.client, "test-model", folder
        )
        self.assertEqual(result, {"action": "open_folder", "path": folder})
        self.client.chat.completions.create.assert_not_called()

    def test_open_prefix_with_absolute_project_path_opens_without_classifier(self):
        folder = codex_router.SOURCE_PATH
        result = codex_router.classify_codex_task(
            self.client, "test-model", f"Mở {folder}"
        )
        self.assertEqual(result, {"action": "open_folder", "path": folder})
        self.client.chat.completions.create.assert_not_called()

    def test_terminal_request_is_directly_routed_to_terminal(self):
        for text in ("giúp mình mở terminal", "hãy mở terminal"):
            with self.subTest(text=text):
                result = codex_router.classify_codex_task(
                    self.client, "test-model", text
                )
                self.assertEqual(
                    result, {"action": "open_app", "app": "Terminal"}
                )
        self.client.chat.completions.create.assert_not_called()

    def test_folder_request_is_classified_as_direct_folder_action(self):
        self.set_classifier_response({
            "action": "open_folder",
            "target": "ổ C",
        })
        result = codex_router.classify_codex_task(
            self.client, "test-model", "hãy mở thư mục ổ C"
        )
        self.assertEqual(result, {"action": "open_folder", "target": "ổ C"})

    def test_application_request_is_classified_without_shell_command(self):
        self.set_classifier_response({
            "action": "open_app",
            "app": "Tomino",
        })
        result = codex_router.classify_codex_task(
            self.client, "test-model", "hãy giúp mình mở Tomino"
        )
        self.assertEqual(result, {"action": "open_app", "app": "Tomino"})

    def test_general_website_request_is_routed_to_search_query(self):
        request = (
            "MỞ TRANG WEB CỦA TRƯỜNG ĐẠI HỌC CÔNG NGHIỆP "
            "THÀNH PHỐ HỒ CHÍ MINH"
        )
        query = "official website of Industrial University of Ho Chi Minh City IUH"
        self.set_classifier_response({"action": "open_website", "query": query})
        result = codex_router.classify_codex_task(
            self.client, "test-model", request
        )
        self.assertEqual(result, {
            "action": "open_website",
            "query": query,
        })
        self.client.chat.completions.create.assert_called_once()

    def test_electrical_faculty_request_routes_to_feet_even_when_iuh_named(self):
        request = (
            "MỞ TRANG WEB CỦA KHOA CÔNG NGHỆ ĐIỆN TRƯỜNG "
            "ĐẠI HỌC CÔNG NGHIỆP THÀNH PHỐ HỒ CHÍ MINH"
        )
        query = "official website of Khoa Công nghệ Điện IUH"
        self.set_classifier_response({"action": "open_website", "query": query})
        result = codex_router.classify_codex_task(
            self.client, "test-model", request
        )
        self.assertEqual(result, {
            "action": "open_website",
            "query": query,
        })
        self.client.chat.completions.create.assert_called_once()

    def test_electronics_faculty_does_not_match_electrical_faculty(self):
        request = "Mở trang web Khoa Công nghệ Điện tử IUH"
        result = codex_router.classify_codex_task(
            self.client, "test-model", request
        )
        self.assertIsNone(result)
        self.client.chat.completions.create.assert_called_once()

    def test_arbitrary_faculty_website_request_routes_generically(self):
        request = "HÃY MỞ TRANG WEB CỦA KHOA CÔNG NGHỆ ĐIỆN"
        query = "official website of Khoa Công nghệ Điện"
        self.set_classifier_response({"action": "open_website", "query": query})
        result = codex_router.classify_codex_task(
            self.client, "test-model", request
        )
        self.assertEqual(result, {
            "action": "open_website",
            "query": query,
        })
        self.client.chat.completions.create.assert_called_once()

    def test_website_classifier_cannot_return_model_chosen_url(self):
        self.set_classifier_response({
            "action": "open_website",
            "url": "https://attacker.example",
            "name": "fake official site",
        })
        with self.assertRaises(ValueError):
            codex_router.classify_codex_task(
                self.client, "test-model", "mở website cho example organization"
            )

    def test_song_request_is_routed_to_direct_youtube_search(self):
        self.set_classifier_response({
            "action": "youtube_search",
            "query": "Em của ngày hôm qua",
        })
        result = codex_router.classify_codex_task(
            self.client, "test-model", "mở bài em của ngày hôm qua"
        )
        self.assertEqual(result, {
            "action": "youtube_search",
            "query": "Em của ngày hôm qua",
        })

    def test_direct_adult_website_request_is_blocked_without_model(self):
        for request in (
            "Mở MissAV",
            "Tìm website khiêu dâm",
            "Mở trang web 18+",
        ):
            with self.subTest(request=request):
                result = codex_router.classify_codex_task(
                    self.client, "test-model", request
                )
                self.assertEqual(
                    result, {"action": "blocked_adult_content"}
                )
        self.client.chat.completions.create.assert_not_called()

    def test_classifier_adult_access_decision_is_forwarded_as_block(self):
        self.set_classifier_response({"action": "blocked_adult_content"})
        result = codex_router.classify_codex_task(
            self.client, "test-model", "Open that site"
        )
        self.assertEqual(result, {"action": "blocked_adult_content"})

    def test_model_cannot_sneak_adult_site_into_safe_search_query(self):
        self.set_classifier_response({
            "action": "open_website",
            "query": "official MissAV website",
        })
        result = codex_router.classify_codex_task(
            self.client, "test-model", "Open the official website"
        )
        self.assertEqual(result, {"action": "blocked_adult_content"})

    def test_filtering_policy_request_is_not_blocked_as_access(self):
        request = "Help me block adult websites on this computer"
        self.set_classifier_response({"action": "codex_task"})
        result = codex_router.classify_codex_task(
            self.client, "test-model", request
        )
        self.assertEqual(result, {
            "action": "codex_task",
            "task": request,
        })

    def test_classifier_cannot_invent_commands_or_rewrite_task(self):
        self.set_classifier_response({
            "action": "codex_task",
            "command": "rm -rf ~",
        })
        with self.assertRaises(ValueError):
            codex_router.classify_codex_task(
                self.client, "test-model", "run the requested command"
            )

if __name__ == "__main__":
    unittest.main()
