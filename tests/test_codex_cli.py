import unittest
import threading
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from robot_ui import codex_cli


class CodexCliTests(unittest.TestCase):
    def test_runs_codex_headlessly_inside_workspace_sandbox(self):
        def start(command, **kwargs):
            output = Path(command[command.index("--output-last-message") + 1])
            output.write_text("Task completed", encoding="utf-8")
            process = SimpleNamespace(
                pid=12345,
                returncode=0,
                communicate=lambda **kwargs: ("", ""),
            )
            return process

        with patch.object(
            codex_cli, "find_codex", return_value="/usr/bin/codex"
        ), patch.object(
            codex_cli.subprocess, "Popen", side_effect=start
        ) as popen:
            answer = codex_cli.run_codex_task("mở Firefox")

        self.assertEqual(answer, "Task completed")
        command = popen.call_args.args[0]
        self.assertEqual(command[:2], ["/usr/bin/codex", "exec"])
        self.assertEqual(command[command.index("--model") + 1], "gpt-6-luna")
        self.assertEqual(
            command[command.index("--sandbox") + 1], "workspace-write"
        )
        self.assertIn("--ephemeral", command)
        self.assertTrue(popen.call_args.kwargs["start_new_session"])
        self.assertNotIn("shell", popen.call_args.kwargs)

    def test_empty_or_oversized_task_is_rejected(self):
        with patch.object(codex_cli.subprocess, "Popen") as popen:
            for task in ("", " " * 5, "x" * 2001):
                with self.assertRaises(ValueError):
                    codex_cli.run_codex_task(task)
        popen.assert_not_called()

    def test_codex_prompt_does_not_claim_a_confirmation_was_given(self):
        prompt = codex_cli._codex_prompt("List project tests")
        self.assertIn("no additional UI confirmation is needed", prompt)
        self.assertNotIn("already confirmed", prompt)

    def test_reports_codex_failure(self):
        process = SimpleNamespace(
            pid=12345,
            returncode=2,
            communicate=lambda **kwargs: ("", "sandbox denied"),
        )
        with patch.object(
            codex_cli, "find_codex", return_value="/usr/bin/codex"
        ), patch.object(
            codex_cli.subprocess, "Popen", return_value=process,
        ):
            with self.assertRaisesRegex(RuntimeError, "sandbox denied"):
                codex_cli.run_codex_task("open Firefox")

    def test_cancelled_task_terminates_without_reporting_failure(self):
        process = SimpleNamespace(pid=12345, returncode=None)
        cancel_event = threading.Event()
        cancel_event.set()
        with patch.object(
            codex_cli, "find_codex", return_value="/usr/bin/codex"
        ), patch.object(
            codex_cli.subprocess, "Popen", return_value=process
        ), patch.object(codex_cli, "_terminate_process_group") as terminate:
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                codex_cli.run_codex_task("build project", cancel_event)
        terminate.assert_called_once_with(process)

    def test_opens_encoded_youtube_search_in_firefox_without_terminal(self):
        with patch.object(
            codex_cli.shutil, "which", return_value="/usr/bin/firefox"
        ), patch.object(codex_cli.subprocess, "Popen") as popen:
            codex_cli.open_youtube_search("Em của ngày hôm qua")

        command = popen.call_args.args[0]
        self.assertEqual(command[0], "/usr/bin/firefox")
        self.assertTrue(command[1].startswith("https://www.youtube.com/results?"))
        self.assertIn("search_query=", command[1])
        self.assertIn("safeSearch=strict", command[1])
        self.assertEqual(len(command), 2)

    def test_blocks_adult_youtube_search_before_launching_browser(self):
        with patch.object(codex_cli.subprocess, "Popen") as popen:
            with self.assertRaisesRegex(PermissionError, "Adult-content"):
                codex_cli.open_youtube_search("MissAV adult video")
        popen.assert_not_called()

    def test_reports_missing_firefox(self):
        with patch.object(codex_cli.shutil, "which", return_value=None):
            with self.assertRaises(FileNotFoundError):
                codex_cli.open_youtube_search("Em của ngày hôm qua")

    def test_opens_website_search_on_fixed_google_host(self):
        with patch.object(
            codex_cli.shutil, "which", return_value="/usr/bin/firefox"
        ), patch.object(codex_cli.subprocess, "Popen") as popen:
            codex_cli.open_website_search(
                "official website of Khoa Công nghệ Điện IUH"
            )
        self.assertEqual(
            popen.call_args.args[0],
            [
                "/usr/bin/firefox",
                "https://www.google.com/search?q=official+website+of+"
                "Khoa+C%C3%B4ng+ngh%E1%BB%87+%C4%90i%E1%BB%87n+IUH"
                "&safe=active",
            ],
        )

    def test_blocks_adult_website_search_before_launching_browser(self):
        with patch.object(codex_cli.subprocess, "Popen") as popen:
            with self.assertRaisesRegex(PermissionError, "Adult-content"):
                codex_cli.open_website_search("official MissAV website")
        popen.assert_not_called()

    def test_codex_prompt_forbids_adult_site_access(self):
        prompt = codex_cli._codex_prompt("Open MissAV")
        self.assertIn("Never access, search for, open", prompt)
        self.assertIn("adult/18+", prompt)

    def test_rejects_unapproved_website_url(self):
        with patch.object(codex_cli.subprocess, "Popen") as popen:
            with self.assertRaises(ValueError):
                codex_cli.open_firefox_url("https://attacker.example")
        popen.assert_not_called()

    def test_opens_project_directory_with_ubuntu_file_manager(self):
        with patch.object(
            codex_cli.shutil, "which", return_value="/usr/bin/xdg-open"
        ), patch.object(codex_cli.subprocess, "Popen") as popen:
            codex_cli.open_local_folder(codex_cli.SOURCE_PATH)
        self.assertEqual(
            popen.call_args.args[0],
            ["/usr/bin/xdg-open", codex_cli.SOURCE_PATH],
        )
        self.assertTrue(popen.call_args.kwargs["start_new_session"])

    def test_rejects_folder_outside_project_and_home(self):
        with self.assertRaises(PermissionError):
            codex_cli.open_local_folder("/etc")

    def test_opens_terminal_with_allowlisted_terminal_executable(self):
        with patch.object(
            codex_cli.shutil, "which",
            side_effect=lambda name: (
                "/usr/bin/gnome-terminal"
                if name == "gnome-terminal" else None
            ),
        ), patch.object(codex_cli.subprocess, "Popen") as popen:
            codex_cli.open_desktop_application("Terminal")
        self.assertEqual(
            popen.call_args.args[0], ["/usr/bin/gnome-terminal"]
        )

    def test_opens_only_matching_installed_desktop_entry(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            desktop_file = root / "org.example.Tomino.desktop"
            desktop_file.write_text(
                "[Desktop Entry]\nType=Application\nName=Tomino\nExec=tomino\n",
                encoding="utf-8",
            )
            with patch.object(
                codex_cli, "_desktop_application_roots", return_value=(root,)
            ), patch.object(
                codex_cli.shutil, "which", return_value="/usr/bin/gio"
            ), patch.object(codex_cli.subprocess, "Popen") as popen:
                codex_cli.open_desktop_application("Tomino")
            self.assertEqual(
                popen.call_args.args[0],
                ["/usr/bin/gio", "launch", str(desktop_file)],
            )
            self.assertNotIn("shell", popen.call_args.kwargs)

    def test_missing_application_is_not_run_as_a_command(self):
        with patch.object(
            codex_cli, "_desktop_application_roots", return_value=()
        ), patch.object(
            codex_cli.shutil, "which", return_value="/usr/bin/gio"
        ), patch.object(codex_cli.subprocess, "Popen") as popen:
            with self.assertRaises(FileNotFoundError):
                codex_cli.open_desktop_application("not-installed")
        popen.assert_not_called()


if __name__ == "__main__":
    unittest.main()
