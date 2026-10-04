import unittest

from robot_ui.web_sources import extract_web_sources, format_answer_sources


class WebSourcesTests(unittest.TestCase):
    def test_unsolicited_inline_citations_from_chat_log_are_removed(self):
        answer = "Thầy là giảng viên. ([feet.iuh.edu.vn](https://feet.iuh.edu.vn/giangvien))\n\nTrường ở TP.HCM. ([iuh.edu.vn](https://iuh.edu.vn/vi/?utm_source=openai))"
        self.assertEqual(format_answer_sources(answer, "Giới thiệu về thầy"),
                         "Thầy là giảng viên.\n\nTrường ở TP.HCM.")

    def test_explicit_source_request_retains_links(self):
        answer = "[IUH](https://iuh.edu.vn/)"
        for question in ("Cho tôi nguồn", "Link đâu?", "Website trường là gì?", "Show sources"):
            self.assertEqual(format_answer_sources(answer, question), answer)
        self.assertNotIn("https://", format_answer_sources(answer, "Không cần nguồn"))

    def test_link_label_is_preserved_without_unsolicited_url(self):
        self.assertEqual(format_answer_sources(
            "[Khoa Điện](https://feet.iuh.edu.vn/) đào tạo kỹ sư. citeabc", "Giới thiệu khoa"),
            "Khoa Điện đào tạo kỹ sư.")

    def test_tool_sources_and_citations_are_deduplicated(self):
        output = [
            {"type": "web_search_call", "action": {"sources": [
                {"url": "https://iuh.edu.vn/units", "title": "Units"}]}},
            {"type": "message", "content": [{"annotations": [
                {"type": "url_citation", "url": "https://iuh.edu.vn/units"},
                {"type": "url_citation", "url": "https://feet.iuh.edu.vn/"},
                {"type": "url_citation", "url": "javascript:alert(1)"},
            ]}]},
        ]
        self.assertEqual(extract_web_sources(output), [
            {"url": "https://iuh.edu.vn/units", "title": "Units"},
            {"url": "https://feet.iuh.edu.vn/", "title": "feet.iuh.edu.vn"},
        ])

    def test_no_sources_are_invented_from_answer_text(self):
        self.assertEqual(extract_web_sources([
            {"type": "web_search_call", "action": {"type": "search"}},
            {"type": "message", "content": [{
                "text": "Theo IUH https://iuh.edu.vn/", "annotations": []}]},
        ]), [])
