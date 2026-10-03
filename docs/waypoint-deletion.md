# Quyền xóa waypoint

Giữ cấu trúc JSON object hiện có; khóa ngoài cùng vẫn là tham chiếu lộ trình
và không dùng làm tên hiển thị có thể đổi. Mỗi waypoint thêm:

- `id`: UUID bền vững; điểm cũ sinh UUID5 từ khóa cũ khi đọc, điểm mới UUID4.
  Sau khi lưu, luôn giữ ID đã có. Đổi `display_name`/`aliases` không đổi ID.
- `display_name`: nhãn trên bản đồ/danh sách. Mặc định bằng khóa cũ.
- `deletable`: boolean. Form điểm mới có “Cho phép xóa”, mặc định bật.
- `deletion_permission_pending`: chỉ cho điểm cũ chưa có quyền xóa, ngoài
  phòng X. Mặc định `deletable=false`; xác nhận xóa hỏi rõ cả việc cấp quyền
  cho lần xóa này. Hủy không thay đổi quyền hay dữ liệu. Điểm được lưu với
  `deletable=false` từ form không có marker này và không xóa được.

Phòng mang khóa X (không phân biệt hoa/thường) và ID của các phòng X gốc
luôn bị khóa, kể cả JSON đặt `deletable=true`. Giá trị sai kiểu như chuỗi
`"false"` là lỗi schema. Không thay khóa/ID để đổi tên; dùng `display_name`.
Đây là chính sách của ứng dụng, không phải cơ chế bảo vệ file trước người
có quyền tự sửa toàn bộ file và mã chương trình.

File waypoint hiện tại đã được bổ sung trường, giữ nguyên khóa, tọa độ, map,
alias và các thuộc tính có trước. Loader vẫn đọc file cũ; không tự ghi file
khi đọc. Không chuyển lộ trình sang schema mới. Các lộ trình giữ khóa cũ;
trước khi xóa kiểm tra cả khóa, ID, tên hiển thị và alias để chặn tham chiếu.
Nếu một lộ trình tham chiếu điểm, người dùng phải chỉnh/xóa lộ trình trước.

Xóa trong danh sách: chọn điểm → “Xóa điểm đến” → xác nhận (mặc định Không).
Phòng X/điểm khóa có nút vô hiệu hóa và lý do. Handler kiểm tra lại quyền,
lộ trình và điều hướng đang chạy, kể cả sau hộp xác nhận. Chỉ đổi bộ nhớ,
danh sách, bản đồ và nguồn nhận diện giọng nói sau khi ghi file thành công.
Startup đọc lại waypoint trước nhận diện/gửi goal, nên không dùng alias đã
xóa từ bản cache. Alias trùng nhau không được chọn ngẫu nhiên.

Ghi vào file tạm cùng thư mục, flush/fsync rồi os.replace. Lỗi ghi/replace
không thay dữ liệu trong bộ nhớ/file đích và không báo thành công. Nếu file
bị sửa bên ngoài kể từ lúc mở màn hình, từ chối ghi đè và yêu cầu mở lại.
Đây là kiểm tra snapshot, không phải giao dịch đa tiến trình cho cả hai file;
tránh sửa file waypoint/lộ trình bằng công cụ khác đồng thời lúc xác nhận.

Schema kiểm tra object, khóa/ID trùng, số hữu hạn, quaternion khác 0, map,
alias và boolean. Nếu lỗi: báo rõ điểm/trường, không âm thầm bỏ điểm rồi ghi
đè phần dữ liệu còn lại; màn hình khóa ghi/xóa đến khi sửa file và mở lại.

Kiểm thử không khởi tạo node ROS hoặc gửi goal thật:

```bash
source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests -p 'test_waypoint*.py'
```

Kiểm thử bao phủ migration/ID, xóa/hủy, bảo vệ X, tham chiếu lộ trình,
điều hướng đang dùng điểm, thay đổi trong lúc xác nhận, lỗi schema/lưu file,
cập nhật danh sách/bản đồ/alias và lưu quyền true/false từ form.
