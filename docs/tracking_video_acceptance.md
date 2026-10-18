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

## Bám người bằng depth D435i

Nguồn `realsense` bật đồng thời BGR8 + Z16, mặc định 640×480 @ 30 Hz,
chọn đúng serial, dùng SDK `align(color)` và depth scale của thiết bị để đổi
sang mét. Cài `pyrealsense2` trong cả môi trường UI và ROS; không chạy thêm
RealSense driver/preview khác chiếm cùng camera. Cấu hình độ phân giải/FPS khác
phải được thiết bị hỗ trợ đồng thời ở cả hai stream. `Camera.read()` vẫn trả
`(ok, color)`; `depth_m` và `intrinsics` chỉ thuộc lần read hiện tại.

Tracker lấy mẫu vùng thân giữa bbox (30–70% chiều ngang, 20–60% chiều cao),
median của khoảng vài trăm pixel. Loại 0/NaN/Inf, ngoài `depth_min..depth_max`,
ROI thiếu 50% mẫu hợp lệ hoặc dưới 12 mẫu, và IQR > 0,25 m.
Cần ba mẫu liên tiếp; thay đổi > 0,35 m giữa hai mẫu sẽ dừng và thu lại ba mẫu.
Median cửa sổ ba mẫu giảm dao động, có thêm độ trễ khoảng 0,2 giây ở 10 Hz.
Không điền lỗ depth từ frame cũ. Mất người, depth/TF lỗi hoặc inference quá
`max_frame_age` sẽ gửi vận tốc 0 và yêu cầu hủy goal Nav2, kể cả goal được
server chấp nhận muộn. Khoảng cách báo cáo được xóa khi không hợp lệ.
Khóa ID không chuyển sang người khác trong lúc người đã khóa bị che khuất;
track hết hạn sau 30 frame mất thì có thể chọn lại người.

Dùng intrinsics của depth đã align (cùng hình học ảnh màu) với
`rs2_deproject_pixel_to_point`, rồi TF optical → `base_link`.
Khoảng cách điều khiển/báo cáo là khoảng cách ngang từ gốc `base_link` đến
vùng thân đo được, không phải chiều cao bbox, cũng không phải chiều sâu Z
thô của camera. Optical: x phải/y xuống/z trước; base_link: x trước/y trái/z lên.
Nguồn V4L2/URL giữ ước lượng chiều cao 1,7 m và giả định camera nhìn thẳng,
đồng vị trí với robot; UI báo `estimated`. RealSense lỗi **không fallback**.

### TF và launch

Ưu tiên TF camera từ URDF/robot_state_publisher đã hiệu chuẩn. Mặc định
`camera_frame:=tracking_camera_color_optical_frame`, phải có đường TF đến
`base_link`. Thiếu TF thì vẫn có preview nhưng không chạy.
Nếu chưa có TF, launch có publisher tùy chọn:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select human_following
source install/setup.bash
ros2 launch human_following system.launch.py \
  camera_url:=realsense camera_serial:=SERIAL_THUC \
  enable_motion:=false desired_distance:=1.0 depth_min:=0.3 depth_max:=3.0
```

Để dùng publisher tùy chọn, thêm `publish_camera_tf:=true` và các tham số
`camera_x/y/z` (mét), `camera_roll/pitch/yaw` (radian) đã đo.
Translation là vị trí gốc optical trong base_link; rotation là hướng optical
so với base_link. Giá trị rotation mặc định (-π/2, 0, -π/2) chỉ đúng cho
camera lắp ngang, nhìn thẳng về phía trước. Translation mặc định 0 không phải
hiệu chuẩn robot thật. Không bật publisher này nếu frame đã có publisher khác.

Tham số launch:
- `desired_distance=1.0`, `depth_min=0.3`, `depth_max=3.0` (m).
- `max_linear=0.2` m/s, `max_angular=0.5` rad/s: giới hạn **Direct**.
- `max_frame_age=0.5` s: giới hạn xử lý từ lúc nhận frame đến tính lệnh;
  không phải watchdog độc lập khi SDK/inference bị treo.
- `use_nav2=false`, `nav_frame=odom`. Nav2 cần TF mới từ base_link đến
  frame cố định này; TF động cũ quá max_frame_age bị từ chối.
- UI truyền các tham số trên qua biến môi trường `TRACKING_<TÊN_IN_HOA>`,
  gồm cả `TRACKING_PUBLISH_CAMERA_TF`, `TRACKING_CAMERA_X`, v.v.;
  áp dụng khi khởi động lại theo dõi.

Direct dùng pose trong base_link và dấu góc ROS (người bên phải → quay phải).
Nav2 dùng FollowPath với đường thẳng lấy mẫu mỗi khoảng 0,1 m, đóng băng trong
nav_frame và dừng trước người một khoảng desired_distance. Chờ goal cũ kết thúc
hủy trước khi gửi goal mới. Đây là đường đưa vào controller, chưa phải global
planner tránh vật cản; cần costmap/collision checking và controller Nav2 đã cấu hình.
Giới hạn tốc độ Nav2 phải đặt trong controller Nav2, không lấy từ max_linear của
tracker. Kiểm tra topic vận tốc Nav2 qua bộ điều phối và watchdog của motor:
gửi zero từ following không tự bảo đảm thắng một nguồn Nav2 riêng đang chạy.
Shutdown chờ tối đa 0,5 s cho phản hồi hủy; mất server cần watchdog phía robot.

### Nghiệm thu depth (cần thực hiện trên robot)

1. Tắt chuyển động. Kiểm tra TF và serial; đứng ở 0,5 / 1 / 1,5 / 2 / 3 m,
   giữa ảnh và hai bên. Đo từ base_link đến vùng thân, ghi khoảng cách thực,
   khoảng cách UI, sai số, tỷ lệ frame hợp lệ và độ trễ. Sai số mục tiêu ban đầu
   ≤ max(0,05 m, 5% khoảng cách), điều chỉnh tiêu chí theo nhu cầu vận hành.
2. Kiểm tra người bên phải có y âm trong base_link; thử camera nghiêng/xê dịch
   với TF đã đo. Hai người ở hai khoảng cách: xác nhận lấy depth đúng người.
3. Che cảm biến depth nhưng giữ ảnh màu, thử áo ít texture, ánh sáng mạnh,
   ra ngoài dải đo, mất TF và người bị che. Phải hiện trạng thái dừng, distance
   trống, không dùng số cũ; trở lại cần ba mẫu hợp lệ.
4. Bật Direct với `max_linear:=0.1 max_angular:=0.3` trong khu vực thử;
   kiểm tra giữ khoảng cách, không lùi khi quá gần, không giật khi depth nhảy,
   dừng khi mất người/depth/rút USB. Đo thời gian dừng ở đầu ra motor.
5. Thử Nav2 riêng với giới hạn controller thấp: kiểm tra frame đường,
   khoảng dừng trước người, hủy cả goal đang chạy và goal đang chờ khi depth mất.
6. Giữ mặc định 640×480/30 FPS để bắt đầu; detector chạy tối đa 10 Hz.
   Đo thời gian inference trước khi tăng độ phân giải. Không tăng dải depth
   chỉ để che lỗi thiếu mẫu; đo lại trong điều kiện ánh sáng thực.

Kiểm thử tự động bổ sung:

```bash
source /opt/ros/jazzy/setup.bash
ROS_LOG_DIR=/tmp/tracking-test-logs venv/bin/python -m unittest discover -s tests -p 'test_tracking*.py' -v
```

Nguồn tham khảo:
- [Thông số D435i tại ProE](https://www.proe.vn/d435i-real-sense-depth-camera):
  dải lý tưởng công bố 0,3–3 m; đây là cơ sở chọn dải mặc định, không bảo đảm
  độ chính xác trong mọi điều kiện.
- [Ví dụ align depth to color chính thức của librealsense](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py):
  bật hai stream, đọc depth scale, align vào color và giải phóng pipeline.
