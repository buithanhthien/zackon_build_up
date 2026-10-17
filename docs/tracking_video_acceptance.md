# Nghiệm thu popup Theo dõi

Popup dùng `/tracking/color/image_raw` (`sensor_msgs/Image`, `bgr8`, best effort,
queue depth 1). Node publish chính frame màu đưa vào detector. UI không mở camera;
QTimer chạy ROS executor trên luồng Qt, sao chép QImage trước khi giữ để vẽ.
Không cần thêm cv_bridge để chuyển buffer BGR8 này; sensor_msgs đã có trong package.xml.

Danh sách dùng SDK RealSense để lấy model/serial, không start pipeline. Camera V4L2
được liệt kê từ sysfs, ưu tiên `/dev/v4l/by-id`, rồi `/dev/v4l/by-path`, cuối cùng
`/dev/videoN`. Chỉ liệt kê interface index 0 để tránh các interface metadata phổ biến;
không đảm bảo mọi thiết bị V4L2 đều hỗ trợ ảnh màu. Node kiểm tra frame khi mở.
Môi trường chạy UI cần pyrealsense2 để chọn RealSense theo serial; lỗi SDK được hiện
ngay dưới danh sách. Môi trường ROS cũng cần SDK để mở RealSense.

Đổi lựa chọn áp dụng khi bấm **Khởi động lại theo dõi**. **Dừng theo dõi** gửi SIGINT
cho nhóm tiến trình và chờ trong worker; hết thời gian sẽ tăng lên SIGTERM/SIGKILL.
Node giữ ROS context sống để gửi vận tốc 0 trước khi giải phóng camera. Đóng/Escape
chờ dọn tiến trình xong rồi hủy ROS context riêng của popup. Startup vẫn mở.
Preview báo mất frame sau 3 giây; mất trạng thái ROS sau 5 giây (60 giây khi khởi tạo)
sẽ dừng cả nhóm tiến trình. Lỗi tiến trình được giữ lại sau khi dọn xong.

## Kiểm thử tự động

Chạy với Python có ROS Jazzy, OpenCV, PyQt6 và các dependency của startup:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tests -p test_tracking_recovery.py -v
python3 -m unittest discover -s tests -p test_startup_stability.py -v
```

Bao gồm metadata D435i/nhiều serial, đường dẫn V4L2, không camera/làm mới,
launch theo nguồn/serial, chặn launch khi không có lựa chọn, dừng/chạy lại,
preview sao chép frame/mất frame, vận tốc 0 và giải phóng camera,
dọn tiến trình con thực sự đang giữ khóa tài nguyên, đóng/Escape và startup.
Kiểm thử mô phỏng không xác nhận được việc robot thật nhận vận tốc 0.

## Thử trên robot thật

1. Cố định robot an toàn, để **Cho phép robot di chuyển** tắt. Build/source workspace
   chứa node mới trước khi mở startup.
2. Cắm D435i, làm mới danh sách; kiểm tra model/serial. Nếu có nhiều camera,
   lần lượt chọn và khởi động lại, che ống kính để xác nhận preview đúng camera.
3. Xác nhận video màu liên tục, popup vẫn tương tác được; chuyển động vẫn tắt.
4. Bấm dừng: preview báo đã dừng, popup và startup còn mở, nhóm ROS con kết thúc,
   camera có thể mở lại. Khởi động lại và xác nhận video trở lại.
5. Rút camera khi đang chạy: kiểm tra thông báo lỗi, lệnh 0 trên
   `/cmd_vel_sources/following` và đầu ra điều khiển robot, camera/process được dọn.
   Cắm lại, làm mới rồi khởi động lại.
6. Thử Đóng và Escape khi đang chạy và khi đang dừng: startup còn mở,
   không còn tiến trình theo dõi do popup sở hữu hoặc camera bị giữ.
7. Chỉ thử chuyển động sau khi đã xác minh đường lệnh dừng từ nguồn following
   qua bộ điều phối tới bộ điều khiển động cơ trong điều kiện thử an toàn.
