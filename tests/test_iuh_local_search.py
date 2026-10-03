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

    def test_exact_lecturer_name_prefers_scalar_item_hoang_dinh_khoi(self):
        result = self.search.search(
            "Thông tin về thầy Hoàng Đình Khôi của Khoa Công nghệ Điện"
        )

        self.assertEqual(result.status, "sufficient")
        self.assertEqual(
            result.evidence[0].path,
            "khoa_cong_nghe_dien.bo_mon.3.giang_vien.5",
        )
        self.assertEqual(
            result.evidence[0].value,
            "Tiến Sĩ Hoàng Đình Khôi",
        )


    def test_exact_lecturer_name_prefers_scalar_item_phan_xuan_le(self):
        result = self.search.search(
            "Thông tin về thầy Phan Xuân Lễ của Khoa Công nghệ Điện"
        )

        self.assertEqual(result.status, "sufficient")
        self.assertEqual(
            result.evidence[0].path,
            "khoa_cong_nghe_dien.bo_mon.0.giang_vien.14",
        )
        self.assertEqual(
            result.evidence[0].value,
            "Tiến Sĩ Phan Xuân Lễ",
        )

    def test_person_department_conflict_is_detected(self):
        result = self.search.search(
            "Thông tin về thầy Phan Xuân Lễ của Bộ môn Cung cấp"
        )

        self.assertEqual(result.status, "ambiguous")
        self.assertIn(
            "khoa_cong_nghe_dien.bo_mon.0.giang_vien.14",
            [e.path for e in result.evidence],
        )

    def test_person_correct_department_remains_sufficient(self):
        result = self.search.search(
            "Thông tin về thầy Phan Xuân Lễ của Bộ môn Cơ sở ngành"
        )

        self.assertEqual(result.status, "sufficient")
        self.assertEqual(
            result.evidence[0].path,
            "khoa_cong_nghe_dien.bo_mon.0.giang_vien.14",
        )

    def test_person_phone_does_not_fall_back_to_faculty_phone(self):
        result = self.search.search(
            "Số điện thoại của thầy Trần Thanh Ngọc trưởng khoa "
            "Khoa Công nghệ Điện là bao nhiêu?"
        )

        self.assertEqual(result.status, "insufficient")

        selected_paths = [
            evidence.path
            for evidence in result.evidence
        ]

        self.assertNotIn(
            "khoa_cong_nghe_dien.lien_he_truc_tiep.dien_thoai",
            selected_paths,
        )

    def test_explicit_web_request_requires_web(self):
        self.assertTrue(
            requires_web_for_freshness(
                "Hãy tìm trên mạng số điện thoại của "
                "Khoa Công nghệ Hóa học ở IUH"
            )
        )

    def test_training_office_location_missing_is_insufficient_not_ambiguous(self):
        result = self.search.search(
            "Phòng Đào tạo của Trường Đại học Công nghiệp nằm ở đâu?"
        )

        self.assertEqual(result.status, "insufficient")

    def test_plural_campus_address_query_is_not_treated_as_missing_subject(self):
        result = self.search.search(
            "Cho tôi địa chỉ các cơ sở của IUH"
        )

        self.assertNotEqual(result.status, "insufficient")
        self.assertTrue(result.evidence)

        self.assertTrue(
            any(
                evidence.path.startswith("co_so.")
                and evidence.field == "dia_chi"
                for evidence in result.evidence
            )
        )

    def test_exact_lecturer_evidence_includes_parent_department_context(self):
        result = self.search.search(
            "Bạn có biết thầy Hoàng Đình Khôi của Khoa Công nghệ Điện không?"
        )
        self.assertEqual(result.status, "sufficient")
        lecturer = result.evidence[0]
        self.assertEqual(lecturer.value, "Tiến Sĩ Hoàng Đình Khôi")
        self.assertEqual(lecturer.parent_department_name, "Bộ môn Tự động hóa")
        self.assertEqual(
            lecturer.parent_department_path,
            "khoa_cong_nghe_dien.bo_mon.3.ten",
        )
        self.assertEqual(
            lecturer.parent_department_head,
            "Phó Giáo Sư Tiến Sĩ Ngô Thanh Quyền",
        )

    def test_third_department_uses_human_one_based_order(self):
        result = self.search.search("Bộ môn thứ ba là gì?")
        self.assertEqual(result.status, "sufficient")
        self.assertEqual(result.evidence[0].value, "Bộ môn Thiết bị điện")
        self.assertEqual(result.evidence[0].ordinal_position, 3)
        self.assertEqual(
            result.evidence[0].path,
            "khoa_cong_nghe_dien.bo_mon.2.ten",
        )
        self.assertIn("human_ordinal_department", result.evidence[0].reasons)

    def test_department_number_three_is_not_json_index_three(self):
        result = self.search.search("Bộ môn 3 là bộ môn nào?")
        self.assertEqual(result.status, "sufficient")
        self.assertEqual(result.evidence[0].value, "Bộ môn Thiết bị điện")
        self.assertNotEqual(result.evidence[0].value, "Bộ môn Tự động hóa")


if __name__ == "__main__":
    unittest.main()
