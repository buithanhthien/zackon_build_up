# Thông tin bổ sung cạnh dữ liệu JSON

Khi tìm thấy dữ liệu JSON, ứng dụng giữ hai nhóm bộ nhớ riêng:

- `corrections`: đính chính đúng `json_path` và `base_json_value`; được phép thay
  thế giá trị JSON tương ứng.
- `supplemental_memories`: bản ghi `source=user_correction`, `provenance=user`
  không có scope JSON. Model được phép dùng để bổ sung thuộc tính còn thiếu của
  đúng người/chủ thể, ghi rõ đây là thông tin người dùng cung cấp. Không được dùng
  để thay thế thuộc tính mâu thuẫn với JSON.

Các bản ghi scoped sai path, giá trị gốc đã thay đổi, dữ liệu kiểm thử hoặc không
rõ nguồn không được đưa trở lại qua nhóm bổ sung. Việc xác định cùng chủ thể,
thuộc tính và mâu thuẫn về ngữ nghĩa vẫn phụ thuộc model; bộ lọc bằng code chỉ
bảo đảm điều kiện metadata và tách hai nhóm quyền sử dụng.

Khi dạy thêm, model trả `local_relation`: `override` cho thay đổi thuộc tính có
trong JSON, `supplement` cho thuộc tính mới, `none` nếu không có quan hệ với JSON.
Thông tin bổ sung được lưu không có scope; cập nhật cùng thuộc tính tái sử dụng
document bổ sung, không thay thế document đính chính JSON. Bản ghi bổ sung cũ
đã lưu không cần được dạy lại hoặc sửa trực tiếp trên server.

Log ghi `candidates`, `scoped_or_unfiltered`, `supplements`, `dropped` để phân
biệt kết quả đã lấy được với dữ liệu bị loại. `candidates` gồm cả dữ liệu phiên
hiện tại sau khi hợp nhất. Nếu route memory không có câu trả lời nhưng JSON đủ
dữ liệu, ứng dụng dùng câu trả lời từ JSON.

Xuất dữ liệu và đối chiếu hai nhóm:

```bash
python3 tool/inspect_hindsight_memory.py --hindsight \
  --query 'Cho mình biết thông tin về thầy Hoàng Đình Khôi' \
  --output .local/hindsight-memory-readable.json
```

Kiểm thử offline:

```bash
python3 -m unittest discover -s tests -q
```

Nghiệm thu UI: mở phiên mới, hỏi giới thiệu thầy Khôi rồi hỏi về đồ án; câu trả
lời cần dùng thông tin bổ sung đã lưu và quy nguồn cho người dùng. Kiểm tra thêm
một dữ kiện mâu thuẫn JSON: thông tin bổ sung không được tự thay thế JSON. Các
test mock không bảo đảm model luôn tuân thủ quy tắc ngữ nghĩa này.
