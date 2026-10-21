# Test thang máy

Nút **Test thang máy** mở cửa sổ riêng trong UI. Mở camera bằng **Mở / kiểm tra
camera**, sau đó nhấn **Bắt đầu**. Model cần được cấp quyền sử dụng và đặt trong
`models/elevator_detect/` theo README tại đó. Thiếu model, sai hash, checkpoint
không nạp được hoặc camera chưa có frame mới đều chặn bắt đầu.

Để kiểm tra riêng lỗi `unknown`, mở camera và bật **Chỉ kiểm tra nhận diện tại
chỗ**; thao tác này chỉ chạy detector, không gửi goal hay lệnh vận tốc. Cửa sổ
hiển thị điểm của cả `available` và `unavailable`, ngưỡng và thời gian suy luận.
Không nhấn **Bắt đầu** khi chỉ muốn quan sát model. Dừng/đóng cửa sổ tắt kiểm tra;
khi bắt đầu test chuyển động, chế độ kiểm tra tại chỗ tự tắt trước khi đến waypoint.

`unknown` do điểm thấp được báo riêng với camera/kết quả cũ hoặc frame sai định
dạng/thời gian. Ví dụ `available: 80%, unavailable: 20%` vẫn là `unknown` với
ngưỡng tích hợp 85%; chương trình nguồn dùng mặc định 70% nên có thể hiển thị
`available` cho cùng frame. Ngưỡng dùng chung nằm trong `confidence_threshold`
của manifest; khởi động lại UI để nạp lại cấu hình. Chưa thay đổi mức 85% dựa
trên phản ánh này vì chưa có frame/điểm camera thực tế để hiệu chỉnh.

Luồng: kiểm tra đầu vào → Nav2 đến `X5_elevator` → xin quyền UI qua arbiter và
giữ vận tốc 0 → chờ nhận diện → tiến 2 m → quay trái 180° → nhả quyền.
Model chỉ được nạp một lần, trên worker riêng; trong chuỗi test chuyển động,
inference chỉ bắt đầu sau khi Nav2 báo thành công. Model vẫn được kiểm tra khả
năng forward khi nạp.

Map được xác minh từ `/map_server/map` và tham số `yaml_filename`, đối chiếu cả
occupancy grid với map `X5_19102026` trong repo; không dựa vào tên map trong cấu
hình UI. Thay đổi `/map` sau đó sẽ được kiểm tra lại. TF hiện tại, Nav2 action,
nhánh vận tốc navigation, arbiter, odometry và cả hai lidar phải sẵn sàng.

`unavailable` và `unknown/error` giữ robot đứng yên không timeout nhận diện.
Lỗi camera/detector trong lúc đến waypoint hủy Nav2; mất xác nhận `available`
trong khi tiến/quay hủy toàn bộ chuỗi, không tự chạy lại. Camera phải cung cấp
RGB/BGR với timestamp hợp lệ; frame và kết quả inference quá 1 giây không được
dùng để cho phép chuyển động. Đây có thể là giới hạn cần đo trên CPU robot.

Khoảng cách và góc dùng MotionController/odometry hiện có; timeout chuyển động,
watchdog UI/arbiter, emergency stop và kiểm tra lidar/odom vẫn hoạt động.
Không publish trực tiếp `/cmd_vel`: lệnh giữ/di chuyển đi qua `/cmd_vel_sources/ui`.
**Dừng / Trả quyền manual** và nút dừng hiện có hủy cả goal đang chờ phản hồi,
xóa các bước còn lại, gửi zero và nhả lease. UI chỉ báo manual khả dụng khi Nav2
đã có kết quả cuối, không còn yêu cầu cấp/trả quyền đang chờ và trạng thái mới
của arbiter xác nhận không còn quyền tự động. Khi chưa xác nhận được, cửa sổ
không đóng và vẫn giữ nút Dừng. Manual ở đây là nhả quyền ROS; chưa có phản hồi
phần cứng để chứng minh tay điều khiển vật lý đã hoạt động.

## Kiểm thử offline

```sh
source /opt/ros/jazzy/setup.bash
PYTHONPATH="robot_ui:$PYTHONPATH" python3 -m unittest robot_ui/test_elevator.py -v
```

Bộ test không khởi tạo node điều khiển và không gửi lệnh cho robot. Bao gồm
thiếu/hỏng/chưa cấp quyền model, nhãn hợp lệ/unknown, chờ 30 ngày mô phỏng, lỗi
Nav2, hủy ở mỗi trạng thái, mất available khi chuyển động và tiến/quay theo odom.

## Nghiệm thu trên robot (chưa thực hiện)

1. Chọn khu vực trống có ít nhất 2 m đường tiến và khoảng trống quay, có người
   trực dừng khẩn cấp. Xác minh giới hạn tốc độ và khoảng cách lidar của
   MotionController hiện có phù hợp robot trước khi chạy thực địa.
2. Kiểm tra sai map, thiếu Nav2, model thiếu/hỏng/sai hash, camera mất hoặc cũ:
   Bắt đầu phải bị chặn; quan sát không có chuyển động.
3. Đến waypoint với Nav2; xác nhận detector chưa ra quyết định chuyển động
   trước khi Nav2 báo thành công. Hủy cả trước và sau khi Nav2 nhận goal.
4. Đưa ảnh `unavailable` và `unknown` kéo dài: giữ đứng yên, không tự thoát.
   Thử mất camera/model và Dừng. Xác nhận arbiter nhả quyền trước khi UI báo manual.
5. Đưa `available` liên tục: kiểm tra ba frame khác nhau, tiến đủ 2 m bằng odom
   rồi mới quay 180°. Thử Dừng giữa lúc tiến và giữa lúc quay; không được chạy
   tiếp bước sau. Camera thay đổi góc nhìn lúc quay có thể ngắt chuỗi nếu model
   không còn xác nhận available; đây là hành vi chặn chuyển động có chủ đích.
6. Thử emergency stop, mất lidar/odom, mất heartbeat, teleop giành quyền trong
   các giai đoạn. Xác nhận không có tự tiếp tục và kiểm tra tay manual thực tế.

Chưa có kết quả camera thực địa hay đánh giá nhầm `available`; kiểm thử phần mềm
không thay thế các bước nghiệm thu này.
