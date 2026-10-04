# Lệnh chuyển động tiếng Việt

Luồng mới: transcript hoặc ô nhập chat → `motion_commands.parse_motion` →
`ChatPanel.motion_command(dict)` → `RobotUI.execute_motion` → `RosMotionController`
→ `MotionController`. Không dùng LLM để tự chọn số lượng hoặc đơn vị.
Waypoint vẫn dùng `waypoint_command` và action ROS 2 `/navigate_to_pose` như trước.

## Định dạng

```json
{"intent":"motion","actions":[
  {"type":"rotate","direction":"left","revolutions":3},
  {"type":"move","direction":"forward","duration_seconds":5},
  {"type":"move","direction":"backward","distance_meters":3}
]}
```

- `rotate`: `left`/`right`, đúng một trong `revolutions`, `degrees`.
- `move`: `forward`/`backward`, đúng một trong `duration_seconds`, `distance_meters`.
- `rotate_continuous`: `left`/`right`, không có số lượng; chỉ được ở cuối chuỗi.
- `stop`: không có tham số, đứng riêng; hủy toàn bộ hàng đợi.

Chấp nhận các câu ví dụ trong yêu cầu, “quay sang trái”, số dạng chữ như
“một trăm tám mươi độ”, “một phẩy năm mét”, và dấu phẩy thập phân.
Bộ phân tích chỉ nhận ngữ pháp mệnh lệnh rõ ràng; câu thiếu đơn vị, chuỗi có
một bước không hợp lệ hoặc tham số ngoài giới hạn bị từ chối toàn bộ.
Lệnh chuyển động trực tiếp và dừng không cần wake word. Quy tắc wake word của
waypoint giữ nguyên. Câu hỏi/hội thoại đi qua luồng hội thoại hiện có.

## Bộ thực thi và giới hạn

Executor ROS riêng chạy 20 Hz; Qt không chờ robot hoàn thành. Mỗi bước phát vận
tốc 0 khi hoàn tất rồi mới bắt đầu bước kế tiếp. Đo quãng đường theo odometry
có dấu theo hướng đi và cộng góc yaw có xử lý qua ±π, không quy đổi mét/vòng
thành thời gian chạy ước tính. Bước theo giây dùng đồng hồ monotonic.
Xoay liên tục chỉ kết thúc do lệnh dừng hoặc điều kiện lỗi/an toàn; luôn phát
thông báo. Trạng thái bắt đầu, từng bước, hoàn thành, dừng và lỗi hiện trong chat.

Giới hạn phần mềm mới (cần hiệu chỉnh trên robot, không phải chứng nhận an toàn):

| Tham số | Giới hạn |
| --- | --- |
| Vận tốc đi | 0,10 m/s |
| Vận tốc xoay | 0,314 rad/s, bằng tốc độ xoay localization hiện có |
| Số bước | 12 |
| Mỗi bước xoay hữu hạn | 5 vòng hoặc 1800 độ |
| Mỗi bước đi theo thời gian | 30 giây |
| Mỗi bước đi theo khoảng cách | 5 mét |
| Mất odometry/lidar/heartbeat UI hoặc gián đoạn vòng điều khiển | 0,75 giây |
| Khoảng hở tối thiểu từ cả hai lidar | 0,45 mét |

Đầu vào ROS: `/odomfromSTM32`, `/front_lidar/scan`, `/rear_lidar/scan`, QoS sensor
data. Đầu ra của giao diện: `geometry_msgs/Twist` trên `/cmd_vel_sources/ui`;
`velocity_arbiter` là nguồn cuối cùng duy nhất phát `/cmd_vel`. Cả hai scan phải có dữ liệu
hợp lệ; scan chỉ có NaN/Inf không được xem là vùng trống. Kiểm tra vật cản dùng
khoảng cách nhỏ nhất toàn scan, nên có thể từ chối chuyển động trong chỗ hẹp.
Bước hữu hạn không đạt mục tiêu sau `2 × thời gian danh định + 5 giây` bị hủy.

`navigation_stop`, nút Stop, và đóng UI đều dừng bộ thực thi mới. Lệnh dừng cũng
vô hiệu kết quả classifier waypoint đến muộn. Chọn waypoint mới dừng chuỗi
chuyển động trước khi gọi Nav2. Không phát vận tốc 0 định kỳ khi bộ thực thi mới
đang rảnh, để không can thiệp waypoint.

## Phân xử vận tốc và kiểm kê nguồn

| Nguồn/chế độ | Topic trước đây | Topic đầu vào arbiter | Ưu tiên | Timeout |
| --- | --- | --- | --- | --- |
| Nav2 controller + behavior server | `/cmd_vel` | `/cmd_vel_sources/navigation` | 20 | 0,5 s |
| Human following trực tiếp | `/cmd_vel` | `/cmd_vel_sources/following` | 40 | 0,5 s |
| Chuyển động từ giao diện | `/cmd_vel` | `/cmd_vel_sources/ui` | 60 | 0,5 s |
| Docking: rear controller, custom range, opennav | `/cmd_vel` | `/cmd_vel_sources/docking` | 70 | 0,5 s |
| LocalizationWorker xoay định vị | `/cmd_vel` | `/cmd_vel_sources/localization` | 80 | 0,5 s |
| Tay cầm có deadman | `/cmd_vel` | `/cmd_vel_sources/teleop` | 100 | 0,5 s |

Số lớn hơn có quyền cao hơn. Node `velocity_arbiter` trong `view_robot_pkg` là
arbiter duy nhất; không chạy thêm executable `twist_mux`. File
`src/view_robot/config/twist_mux.yaml` nay chứa tham số **của velocity_arbiter**.
Các node trực tiếp dùng mặc định topic riêng; launch Nav2 và opennav docking
remap `cmd_vel` vào topic riêng. Collision monitor (nếu bật pipeline đó) cũng
được cấu hình đầu ra vào nhánh navigation. Không thay đổi subscriber `/cmd_vel`
của STM32, mô phỏng hoặc công cụ giám sát.

Human following ở chế độ `use_nav2=True` dùng action FollowPath, nên vận tốc
đi qua nhánh navigation. Các implementation docking là lựa chọn thay thế;
chỉ chạy một bộ điều khiển docking cùng lúc. Không bật collision monitor như
một nguồn song song với controller khi chưa nối đúng pipeline đầu vào của nó.

### Xin quyền và chuyển quyền

1. Giao diện gọi bất đồng bộ service `/velocity_arbiter/ui_control`
   (`std_srvs/SetBool`, `data=true`) để xin quyền. Publisher Nav2 đang rảnh
   không chặn yêu cầu. Arbiter từ chối nếu đang có nguồn sở hữu hoặc action Nav2
   accepted/executing/canceling, kể cả lúc vận tốc Nav2 bằng 0.
2. Arbiter theo dõi status của NavigateToPose, NavigateThroughPoses, FollowPath,
   Spin, BackUp và DriveOnHeading. UI chỉ bắt đầu khi service chấp nhận và
   heartbeat `/velocity_arbiter/state` xác nhận `owner=ui`, `ui_granted=true`.
3. Mỗi mẫu vận tốc hợp lệ gia hạn quyền 0,5 giây. UI không gửi mẫu trước khi
   được cấp quyền. Lower-priority samples bị bỏ qua, không lưu để phát lại.
4. Teleop/localization/docking có thể giành quyền của UI. Khi mất quyền, UI
   dừng action và hủy cả chuỗi; không tự tiếp tục khi nguồn ưu tiên cao trả quyền.
   Lệnh UI đến muộn sau khi thu hồi quyền cũng bị bỏ qua.
5. Hoàn thành, Stop hoặc lỗi trả quyền qua service (`data=false`). Stop khi
   service còn chờ hủy yêu cầu và trả ngay quyền nếu response chấp nhận đến muộn.
6. Không có mẫu mới trong 0,5 giây: arbiter thu hồi quyền, phát zero ở 20 Hz.
   Đồng hồ watchdog là monotonic/steady, không phụ thuộc `/clock` mô phỏng.
   Sau khi trả quyền, nguồn khác chỉ được chạy bằng **mẫu mới**, không bằng mẫu
   đã bị chặn trước đó. Nav2/following đang tiếp tục phát có thể lấy lại quyền
   bằng mẫu mới; arbiter không tự hủy action của các node đó.
7. Tay cầm chỉ phát vận tốc khi giữ deadman. Giữ deadman với cần ở giữa vẫn
   giữ quyền và phát zero. Khi nhả nút, gửi `/velocity_arbiter/release/teleop`
   (`std_msgs/Empty`) một lần; khi tay cầm mất kết nối, timeout xử lý.

Heartbeat arbiter là JSON `std_msgs/String`, có `owner`, `ui_granted`, `healthy`,
`navigation_busy`, `emergency_stop`. UI dừng nếu heartbeat cũ quá 0,3 giây.
Guard vẫn kiểm tra `/cmd_vel` có đúng một publisher tên `velocity_arbiter`;
các publisher Nav2 lúc này nằm trên nhánh riêng nên không gây chặn giả.
Arbiter đánh dấu không khỏe nếu xuất hiện nguồn ghi thẳng `/cmd_vel`, nhiều
arbiter hoặc không có subscriber đầu ra; khi đó hủy quyền và phát zero.
Kiểm tra ROS graph không thể ngăn một node cấu hình sai tiếp tục ghi trực tiếp;
phải sửa/remap node đó, không nới guard.

### Dừng khẩn cấp và giới hạn phần cứng

Có đầu vào phần mềm `/velocity_arbiter/emergency_stop` (`std_msgs/Bool`): `true`
hủy quyền, chặn mọi nguồn và phát zero; `false` mở khóa, không khôi phục lệnh
đã hủy. Trạng thái này nằm trong tiến trình, không được lưu qua restart.
Đây là **điểm nối phần mềm**, chưa được nối với nút E-stop vật lý của robot.
Nút Stop trên UI vẫn hủy Nav2 và chuỗi UI; không thay thế E-stop toàn hệ thống.

Nếu **nguồn điều khiển** mất kết nối, arbiter còn sống sẽ dừng sau timeout.
Nếu chính arbiter chết hoặc đường truyền cuối cùng tới STM32 mất, cần watchdog
trong firmware để dừng động cơ; repository chưa chứa phần firmware này nên
chưa xác minh được. Các kiểm thử không chứng nhận khoảng phanh, footprint,
độ phủ lidar, độ trượt hay ưu tiên nút E-stop phần cứng.

## Build và áp dụng

Đã kiểm chứng build cả ba package vào `/tmp/zackon-arbiter-validation`;
chưa thay thế các tiến trình robot đang chạy. Để áp dụng vào workspace:

```bash
cd /home/khoaiuh/zackon_build_up
source /opt/ros/jazzy/setup.bash
colcon build --packages-select view_robot_pkg human_following lidar_dock_detector
source install/setup.bash
```

Dừng các tiến trình nguồn vận tốc cũ và khởi động lại UI, Nav2, teleop,
following/docking cần dùng từ workspace đã build. Chỉ restart UI sẽ vẫn thấy
controller/behavior server cũ ghi thẳng `/cmd_vel` và bị guard từ chối đúng.
Không chạy bản cũ và bản mới song song.

`NAV2_BRINGUP.launch.py`/`zackon_navigation.launch.py` và
`MAP_GENERATING.launch.py` tự chạy arbiter mặc định. Nếu đã có arbiter độc lập,
truyền `start_velocity_arbiter:=false` vào launch đó để tránh tạo node thứ hai.
Các launch teleop/following/docking không tự tạo thêm arbiter. Khi dùng riêng:

```bash
ros2 launch view_robot_pkg velocity_arbiter.launch.py
```

Kiểm tra graph **trước khi chạy trên robot thật**:

```bash
ros2 topic info --verbose /cmd_vel
ros2 topic info --verbose /cmd_vel_sources/navigation
ros2 topic info --verbose /cmd_vel_sources/ui
ros2 topic echo /velocity_arbiter/state
```

Kỳ vọng `/cmd_vel` có đúng **1 publisher: velocity_arbiter**. Nhánh navigation
có thể có nhiều publisher controller/behavior server. Khi Nav2 rảnh,
`healthy=true`, `owner=""`, `navigation_busy=false`; UI có thể xin quyền.
Nếu `healthy=false`, kiểm tra nguồn ghi thẳng đầu ra, arbiter trùng và bộ nhận
cuối. Nếu UI báo Nav2 bận, hủy action rồi đợi status hoàn tất trước khi thử lại.

## Kiểm thử

`tests/test_motion_commands.py`: parser, kiểm tra schema, chuỗi, odometry,
xoay liên tục, dừng, lỗi publish, mất cảm biến, vật cản và timeout.
`tests/test_motion_integration.py`: dispatch UI, nhập văn bản khi AI bận, dừng
chung, classifier đến muộn/sai lệnh, phân tách waypoint và guard ROS adapter.
Bộ hồi quy dùng thêm startup destinations, waypoint deletion, conversation policy
và voice recording flow.

```bash
source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=offscreen venv/bin/python -m unittest discover -s tests -p 'test_motion*.py'
```

### Bộ kiểm thử arbiter

- `tests/test_velocity_arbiter.py`: ưu tiên, timeout, quyền UI, không phát lại mẫu
  cũ, zero từ nguồn rảnh, deadman và hook dừng khẩn cấp.
- `tests/test_velocity_arbiter_ros.py`: graph ROS thực với receiver/cảm biến giả
  và sáu publisher Nav2 giả lập; không khởi động driver robot hoặc Nav2 thật.
- Kết quả kiểm chứng: **74 kiểm thử unit/UI/hồi quy đạt**; **10 kiểm thử ROS
  trên domain localhost riêng đạt**. Câu “KHANG ĐI LÙI HAI GIÂY” phát đúng
  vận tốc lùi rồi zero; `/cmd_vel` có duy nhất arbiter. Các ca Nav2 bận,
  Nav2 có action nhưng đứng yên, teleop giành quyền, Stop lúc xin quyền,
  Stop khi xoay liên tục, mất mẫu UI/navigation và publisher chưa remap đều đạt.
- Ba package build thành công. Package docking còn cảnh báo format `RCLCPP_INFO`
  có sẵn trong `lidar_intensity_dock.cpp`; không thuộc thay đổi routing này.

Chạy kiểm thử ROS trong domain **181 chỉ dành cho kiểm thử**, bảo đảm không có
robot/driver nào cấu hình domain này:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ROS_LOG_DIR=/tmp/zackon-arbiter-test-logs RUN_ARBITER_ROS_TESTS=1 \
venv/bin/python -m unittest discover -s tests -p test_velocity_arbiter_ros.py -v
```
