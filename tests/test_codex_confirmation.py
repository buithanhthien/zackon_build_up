import ast
import os
from pathlib import Path
import unittest
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication, QWidget


class AutomaticChatActionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source_path = (
            Path(__file__).resolve().parents[1]
            / "robot_ui"
            / "chat_panel_widget.py"
        )
        module = ast.parse(source_path.read_text(encoding="utf-8"))
        chat_panel = next(
            node for node in module.body
            if isinstance(node, ast.ClassDef) and node.name == "ChatPanel"
        )
        methods = [
            node for node in chat_panel.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "_on_system_action_ready"
        ]
        methods[0].decorator_list = []
        namespace = {
            "open_desktop_application": Mock(),
            "open_firefox_url": Mock(),
            "open_website_search": Mock(),
            "open_local_folder": Mock(),
            "open_youtube_search": Mock(),
            "QThread": Mock(),
            "_CodexTaskWorker": Mock(),
        }
        class_node = ast.ClassDef(
            name="ChatPanelUnderTest",
            bases=[ast.Name(id="QWidget", ctx=ast.Load())],
            keywords=[],
            body=methods,
            decorator_list=[],
        )
        module = ast.Module(
            body=[class_node],
            type_ignores=[],
        )
        ast.fix_missing_locations(module)
        code = compile(module, str(source_path), "exec")
        namespace["QWidget"] = QWidget
        exec(code, namespace)
        cls.panel_class = namespace["ChatPanelUnderTest"]
        cls.open_desktop_application = namespace["open_desktop_application"]
        cls.open_firefox_url = namespace["open_firefox_url"]
        cls.open_website_search = namespace["open_website_search"]
        cls.open_local_folder = namespace["open_local_folder"]
        cls.open_youtube_search = namespace["open_youtube_search"]
        cls.thread_factory = namespace["QThread"]
        cls.worker_factory = namespace["_CodexTaskWorker"]
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        type(self).open_youtube_search.reset_mock()
        type(self).open_local_folder.reset_mock()
        type(self).open_desktop_application.reset_mock()
        type(self).open_firefox_url.reset_mock()
        type(self).open_website_search.reset_mock()
        type(self).thread_factory.reset_mock()
        type(self).worker_factory.reset_mock()
        self.panel = type(self).panel_class()
        self.panel._language = "vi"
        self.panel._on_response = Mock()
        self.panel._on_error = Mock()
        self.panel._on_codex_thread_finished = Mock()
        self.panel._codex_thread = None
        self.panel._codex_worker = None

    def test_youtube_request_opens_search_without_confirmation(self):
        self.panel._on_system_action_ready({
            "action": "youtube_search",
            "query": "Em của ngày hôm qua",
        })

        type(self).open_youtube_search.assert_called_once_with(
            "Em của ngày hôm qua"
        )
        self.panel._on_response.assert_called_once_with(
            "Đã mở Firefox tới kết quả YouTube cho “Em của ngày hôm qua”."
        )
        self.panel._on_error.assert_not_called()

    def test_project_path_opens_folder_manager_without_confirmation(self):
        folder = "/home/khoaiuh/zackon_build_up"
        self.panel._on_system_action_ready({
            "action": "open_folder",
            "path": folder,
        })

        type(self).open_local_folder.assert_called_once_with(folder)
        self.panel._on_error.assert_not_called()

    def test_terminal_opens_directly_without_confirmation(self):
        self.panel._on_system_action_ready({
            "action": "open_app",
            "app": "Terminal",
        })

        type(self).open_desktop_application.assert_called_once_with("Terminal")
        self.panel._on_error.assert_not_called()

    def test_website_search_opens_without_confirmation(self):
        query = "official website of Khoa Công nghệ Điện IUH"
        self.panel._on_system_action_ready({
            "action": "open_website",
            "query": query,
        })

        type(self).open_website_search.assert_called_once_with(
            query
        )
        self.panel._on_response.assert_called_once()
        self.panel._on_error.assert_not_called()

    def test_invalid_website_search_does_not_open_anything(self):
        self.panel._on_system_action_ready({
            "action": "open_website",
            "query": "",
        })

        type(self).open_website_search.assert_not_called()
        self.panel._on_error.assert_called_once()

    def test_blocked_adult_content_request_is_refused_without_browser(self):
        self.panel._on_system_action_ready({
            "action": "blocked_adult_content",
        })

        type(self).open_website_search.assert_not_called()
        type(self).open_youtube_search.assert_not_called()
        self.panel._on_response.assert_called_once_with(
            "Mình không thể mở hoặc tìm kiếm website/nội dung người lớn."
        )

    def test_codex_task_starts_in_background_without_confirmation(self):
        worker = Mock()
        thread = Mock()
        type(self).worker_factory.return_value = worker
        type(self).thread_factory.return_value = thread

        self.panel._on_system_action_ready({
            "action": "codex_task",
            "task": "List project tests",
        })

        type(self).worker_factory.assert_called_once_with("List project tests")
        type(self).thread_factory.assert_called_once_with(self.panel)
        worker.moveToThread.assert_called_once_with(thread)
        thread.start.assert_called_once_with()
        self.panel._on_error.assert_not_called()


if __name__ == "__main__":
    unittest.main()
