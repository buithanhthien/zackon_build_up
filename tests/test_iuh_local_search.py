import unittest
from pathlib import Path

from robot_ui.iuh_local_search import IuhLocalSearch, requires_web_for_freshness


HERE = Path(__file__).resolve().parent
DATABASE_CANDIDATES = (
    HERE.parent / "robot_ui" / "iuh_database.json",
    HERE / "iuh_database.json",
)
DATABASE = next(path for path in DATABASE_CANDIDATES if path.exists())


class IuhLocalSearchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.search = IuhLocalSearch(DATABASE)

    def test_training_office_email_has_verifiable_path(self):
        result = self.search.search("Email phòng đào tạo là gì?")
        self.assertEqual(result.status, "sufficient")
        self.assertEqual(result.evidence[0].path, "lien_he.phong_dao_tao")
        self.assertEqual(result.evidence[0].value, "phongdaotao@iuh.edu.vn")

    def test_pham_van_chieu_address_has_verifiable_path(self):
        result = self.search.search("Địa chỉ cơ sở Phạm Văn Chiêu ở đâu?")
        self.assertEqual(result.status, "sufficient")
        self.assertEqual(result.evidence[0].path, "co_so.co_so_pham_van_chieu.dia_chi")

    def test_missing_student_count_is_insufficient(self):
        result = self.search.search("IUH có bao nhiêu sinh viên?")
        self.assertEqual(result.status, "insufficient")
        self.assertEqual(result.evidence, ())

    def test_bare_address_is_ambiguous(self):
        result = self.search.search("Địa chỉ là gì?")
        self.assertEqual(result.status, "ambiguous")
        self.assertGreater(len(result.evidence), 1)
        self.assertTrue(all(item.field == "dia_chi" for item in result.evidence))

    def test_faculty_head_uses_faculty_context(self):
        result = self.search.search("Trưởng khoa Công nghệ Điện là ai?")
        self.assertEqual(result.status, "sufficient")
        self.assertEqual(
            result.evidence[0].path,
            "khoa_cong_nghe_dien.ban_lanh_dao.truong_khoa",
        )

    def test_freshness_detector_routes_current_questions_to_web(self):
        self.assertTrue(requires_web_for_freshness("Tin mới nhất của IUH là gì?"))
        self.assertTrue(requires_web_for_freshness("Thời tiết hôm nay thế nào?"))
        self.assertFalse(requires_web_for_freshness("Email phòng đào tạo là gì?"))

    def test_reload_after_json_file_changes(self):
        import json
        import tempfile
        import time

        original = {"lien_he": {"phong_dao_tao": "old@iuh.edu.vn"}}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "iuh_database.json"
            path.write_text(json.dumps(original, ensure_ascii=False), encoding="utf-8")
            search = IuhLocalSearch(path)
            first = search.search("Email phòng đào tạo là gì?")
            self.assertEqual(first.evidence[0].value, "old@iuh.edu.vn")

            updated = {"lien_he": {"phong_dao_tao": "new@iuh.edu.vn"}}
            time.sleep(0.002)
            path.write_text(json.dumps(updated, ensure_ascii=False), encoding="utf-8")
            second = search.search("Email phòng đào tạo là gì?")
            self.assertEqual(second.evidence[0].value, "new@iuh.edu.vn")

    def test_count_can_be_derived_from_json_list(self):
        result = self.search.search("Bộ môn Tự động hóa có bao nhiêu giảng viên?")
        self.assertEqual(result.status, "sufficient")
        self.assertEqual(
            result.evidence[0].path,
            "khoa_cong_nghe_dien.bo_mon.3.giang_vien",
        )
        self.assertEqual(result.evidence[0].value, 22)
        self.assertIn("derived_count_from_list", result.evidence[0].reasons)

    def test_plural_list_query_prefers_list_path(self):
        result = self.search.search("Các chương trình đào tạo của Khoa Công nghệ Điện là gì?")
        self.assertEqual(result.status, "sufficient")
        self.assertEqual(
            result.evidence[0].path,
            "khoa_cong_nghe_dien.chuong_trinh_dao_tao",
        )
        self.assertIsInstance(result.evidence[0].value, list)


if __name__ == "__main__":
    unittest.main()
