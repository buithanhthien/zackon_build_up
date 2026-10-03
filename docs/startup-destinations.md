# Điểm đến trên startup

Nút **Điểm đến** mở `DestinationDialog` ngay trên startup, với bốn chức năng:
Tải bản đồ, Địa điểm, Tạo lộ trình, Tạo địa điểm mới và nút Đóng.
Dialog không sở hữu ROS node, tiến trình robot, ChatPanel hoặc bản đồ.
Đóng bằng nút Đóng, Escape hoặc nút X chỉ đóng dialog. Mở lại dùng cùng cửa sổ.

`waypoint_dialogs.py` chứa các dialog quản lý điểm đến được startup sử dụng.
`waypoint_map_widget.py` chứa map widget dùng chung. Startup không import hoặc
khởi chạy layout waypoint riêng.

Icon bản đồ ở thanh trên startup mở/đóng cửa sổ bản đồ độc lập. Bản đồ hiển thị
waypoint thuộc bản đồ hiện tại và cập nhật pose AMCL; đóng cửa sổ không hủy Nav2.

Danh sách địa điểm đọc lại `waypoints.json` khi mở và trước khi gửi goal.
Tạo địa điểm lấy pose AMCL tại lúc xác nhận, lưu qua `waypoint_store` với kiểm tra
schema và thay file nguyên tử. Pose thiếu, tên trùng hoặc file lỗi đều có thông báo.
Lộ trình dùng chung `multi_waypoints.json`; startup kiểm tra các điểm thuộc bản đồ
hiện tại trước khi chạy qua hàng đợi Nav2 sẵn có. Các callback và timer của hành
trình cũ không được phép thay đổi hành trình mới.

Tải bản đồ vẫn dùng `LoadMapDialog` và `update_map_files` hiện có (cập nhật cấu hình
và build workspace). Không thêm cơ chế đổi map trực tiếp trong Nav2 đang chạy.
Sau khi tải thành công, startup đọc lại waypoint, làm mới cửa sổ bản đồ và chờ pose
AMCL mới trước khi cho lưu địa điểm.

## Kiểm thử

```bash
source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=offscreen venv/bin/python -m unittest discover -s tests -p 'test_waypoint*.py'
QT_QPA_PLATFORM=offscreen venv/bin/python -m unittest discover -s tests -p 'test_startup_destinations.py'
```

Test dùng Qt offscreen, file tạm và mock Nav2, không khởi động tiến trình robot.
Trên robot, kiểm tra thêm: chọn map theo quy trình Nav2 hiện tại, tạo địa điểm từ
AMCL, chọn điểm để đi, tạo/chạy lộ trình nhiều điểm; trong lúc đang đi, đóng/mở cả
hai cửa sổ để xác nhận startup và Nav2 tiếp tục hoạt động.
