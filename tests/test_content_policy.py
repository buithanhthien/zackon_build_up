import unittest

from robot_ui.content_policy import (
    contains_adult_content_marker,
    is_blocked_adult_access_request,
    is_blocked_adult_filmography_request,
)


class ContentPolicyTests(unittest.TestCase):
    def test_detects_adult_site_names_and_vietnamese_terms(self):
        self.assertTrue(contains_adult_content_marker("MissAV"))
        self.assertTrue(contains_adult_content_marker("phim người lớn"))

    def test_blocks_direct_adult_access_requests(self):
        self.assertTrue(is_blocked_adult_access_request("Mở MissAV"))
        self.assertTrue(is_blocked_adult_access_request("Tìm phim 18+"))

    def test_does_not_block_request_to_configure_content_filtering(self):
        self.assertFalse(
            is_blocked_adult_access_request(
                "Help me block adult websites on this computer"
            )
        )

    def test_blocks_filmography_followup_from_adult_performer_context(self):
        previous_answer = (
            "Ishikawa Mio là nữ diễn viên phim người lớn người Nhật Bản."
        )
        followup = "cho tôi những bộ phim cô ấy đóng"
        self.assertTrue(
            is_blocked_adult_filmography_request(followup, previous_answer)
        )

    def test_blocks_filmography_when_adult_subject_is_in_current_question(self):
        question = "List the filmography of an adult actress"
        self.assertTrue(
            is_blocked_adult_filmography_request(question)
        )

    def test_allows_ordinary_actor_filmography(self):
        question = "Cho tôi những bộ phim cô ấy đóng"
        context = "Cô ấy là một nữ diễn viên."
        self.assertFalse(
            is_blocked_adult_filmography_request(question, context)
        )


if __name__ == "__main__":
    unittest.main()
