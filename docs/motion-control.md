# Lệnh chuyển động tiếng Việt

Luồng mới: transcript hoặc ô nhập chat → `motion_commands.parse_motion` →
`ChatPanel.motion_command(dict)` → `RobotUI.execute_motion` → `RosMotionController`
→ `MotionController` (trực tiếp) hoặc `NavigationTask` (Nav2).
Không dùng LLM để tự chọn số lượng hoặc đơn vị.
Waypoint vẫn dùng `waypoint_command` và action ROS 2 `/navigate_to_pose` như trước.

## Định dạng

```json
{"intent":"motion","actions":[
  {"type":"rotate","direction":"left","revolutions":3},
  {"type":"move","direction":"forward","duration_seconds":5},
  {"type":"move","direction":"backward","duration_seconds":3}
]}
```

- `rotate`: `left`/`right`, đúng một trong `revolutions`, `degrees`.
- `move`: `forward`/`backward`, parser dùng `duration_seconds` cho lệnh theo giây.
  Payload trực tiếp cũ với `distance_meters` vẫn được hỗ trợ nội bộ.
- `navigate_move`: `forward`/`backward`, chỉ `distance_meters`, phải đứng riêng.
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
| Vận tốc đi trực tiếp | 1,0 m/s |
| Vận tốc xoay trực tiếp | 0,628 rad/s |
| Số bước | 12 |
| Mỗi bước xoay hữu hạn | 5 vòng hoặc 1800 độ |
| Mỗi bước đi theo thời gian | 30 giây |
| Mỗi bước đi theo khoảng cách | 5 mét |
| Mất odometry/lidar hoặc gián đoạn vòng điều khiển trực tiếp | 2 giây |
| Mất heartbeat UI | 0,75 giây |
| Khoảng hở tối thiểu từ cả hai lidar khi đi trực tiếp | 0,10 mét |

Đầu vào ROS: `/odomfromSTM32`, `/front_lidar/scan`, `/rear_lidar/scan`, QoS sensor
data. Đầu ra của giao diện: `geometry_msgs/Twist` trên `/cmd_vel_sources/ui`;
`velocity_arbiter` là nguồn cuối cùng duy nhất phát `/cmd_vel`. Cả hai scan phải có dữ liệu
hợp lệ; scan chỉ có NaN/Inf không được xem là vùng trống. Kiểm tra vật cản dùng
khoảng cách nhỏ nhất toàn scan, nên có thể từ chối chuyển động trong chỗ hẹp.
Bước hữu hạn không đạt mục tiêu sau `2 × thời gian danh định + 5 giây` bị hủy.

`navigation_stop`, nút Stop, và đóng UI đều dừng bộ thực thi mới. Lệnh dừng cũng
vô hiệu kết quả classifier waypoint đến muộn. Waypoint, lệnh trực tiếp và lệnh né vật cản mới bị từ chối khi
có tác vụ đang chạy/chờ cấp quyền/chờ nhận goal/chờ hủy; cần Stop rồi chờ kết quả cuối. Không phát vận tốc 0 định kỳ khi bộ thực thi mới
đang rảnh, để không can thiệp waypoint.

## Tiến/lùi có né vật cản qua Nav2

Lệnh tiến/lùi theo **mét mặc định dùng Nav2**, không cần nói “né vật cản”.
Hậu tố này vẫn được chấp nhận nếu người dùng nói thêm:

| Lệnh | Thực thi |
| --- | --- |
| `tiến 2 mét` | `navigate_move`: mục tiêu 2 m phía trước, Nav2 lập đường |
| `tiến 2 mét né vật cản` | `navigate_move`: mục tiêu 2 m phía trước, Nav2 lập đường |
| `lùi một phẩy năm mét` | `navigate_move`: mục tiêu 1,5 m phía sau |
| `tiến 2 giây` | `move`: phát vận tốc trực tiếp, lidar kiểm tra điều kiện dừng |

```json
{"intent":"motion","actions":[
  {"type":"navigate_move","direction":"backward","distance_meters":1.5}
]}
```

Khoảng cách phải hữu hạn và `0 < d ≤ 5` mét. Chấp nhận `m` hoặc `mét`,
số dạng chữ và dấu phẩy thập phân. Không nhận giây, tốc độ, frame tùy chọn,
chuỗi ghép với bước khác hoặc thông số dư. Câu thiếu số/đơn vị, mơ hồ hoặc
một bước sai bị từ chối toàn bộ. Lệnh theo giây và lệnh xoay giữ nguyên ý nghĩa;
lệnh theo mét chuyển sang Nav2 mặc định và phải đứng riêng, không ghép chuỗi.
Parser chạy
trước classifier; kết quả motion của LLM chỉ được chấp nhận nếu bằng kết quả
parser trên input gốc, nên LLM không thể tự tạo tọa độ/số lượng/đơn vị.

Adapter lấy TF mới nhất `map ← base_link`, từ chối TF thiếu/cũ quá 0,75 s,
ở tương lai hoặc quaternion không hợp lệ. Không lấy trực tiếp tọa độ odometry
làm tọa độ map, cũng không dùng pose AMCL lưu lâu trong UI. Với pose `(x,y,θ)`:

- Tiến: `(x + d cos θ, y + d sin θ)`.
- Lùi: `(x − d cos θ, y − d sin θ)`.

Goal giữ yaw hiện tại, đặt frame `map` và timestamp theo đồng hồ ROS.
Khoảng cách là độ lệch mục tiêu, không phải tổng chiều dài đường đi; Nav2 có
thể đi đường vòng. “Lùi” chỉ có nghĩa mục tiêu phía sau, **không đảm bảo di
chuyển vật lý theo chiều lùi trong suốt hành trình**. Cấu hình hiện tại dùng
SmacPlanner2D, RotationShimController bọc MPPI; MPPI có `vx_min: -0.6` nhưng
Rotation Shim có thể quay theo hướng đường đi và `PreferForwardCritic` ưu tiên
tiến. Muốn bắt buộc chạy lùi cần cấu hình và kiểm thử planner/controller riêng.

Luồng waypoint dùng node ROS của UI; lệnh tương đối dùng executor ROS độc lập
của adapter. Cả hai gửi action `/navigate_to_pose`, để planner/controller và
costmap Nav2 xử lý đường. Global costmap dùng `map`, local costmap dùng `odom`,
robot frame `base_link`, obstacle scans `/merged`. Launch remap controller và
behavior server vào `/cmd_vel_sources/navigation`; arbiter phát `/cmd_vel` cuối.
Lệnh tương đối kiểm tra publisher controller ở nhánh này và trạng thái arbiter,
không gọi service xin quyền UI và không tự phát `Twist`. Khi nguồn khác giành
quyền hoặc mất heartbeat UI/arbiter, adapter yêu cầu hủy goal; không tự tiếp tục.

Chat báo chờ nhận goal, đang điều hướng, bị từ chối, thành công, thất bại,
timeout và đang hủy/đã hủy. Timeout nhận goal 5 s; timeout hành trình 120 s tính
từ acceptance, theo monotonic. Timeout yêu cầu hủy; phản hồi cancel service
không được coi là robot đã dừng. Giữ trạng thái bận đến kết quả cuối, kể cả khi
goal được nhận muộn sau Stop; thử hủy lại mỗi 2 s khi cần. Dừng bằng chat,
`navigation_stop`, nút Stop và đóng UI đều hủy goal tương đối và waypoint.
Waypoint giữ handle đến kết quả cuối thay vì xóa ngay khi yêu cầu cancel.
Đóng UI chờ tối đa 5 s cho mỗi luồng; nếu chưa xác nhận kết thúc thì giữ cửa sổ
và executor mở để tiếp tục xử lý phản hồi. Có thể nhấn Stop để thử hủy waypoint lại.

### Kiểm thử cho lệnh Nav2 tương đối

```bash
source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_motion_commands.py tests/test_motion_navigation.py tests/test_motion_integration.py tests/test_startup_destinations.py tests/test_velocity_arbiter.py -q
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 RUN_ARBITER_ROS_TESTS=1 python3 -m pytest tests/test_velocity_arbiter_ros.py -q
```

Test đơn vị kiểm tra ngữ pháp, payload, các yaw/hướng, TF mới/cũ/thiếu, nhánh
vận tốc, phản hồi nhận/từ chối, kết quả cuối, timeout, cancel đến muộn và đóng UI.
Test DDS dùng domain 181 tách biệt, action server/TF/sensor/vận tốc giả lập để
kiểm tra goal đi qua navigation, UI không xin quyền/phát Twist và đóng UI chờ
kết quả hủy. Server giả không có planner/costmap, nên chưa chứng minh né vật cản.

Kiểm thử thực địa hoặc mô phỏng Nav2 đầy đủ còn cần: đặt vật cản trên đường
thẳng đến goal (ngoài footprint), chạy tiến/lùi né vật cản với các yaw, xác nhận
đường vòng hoặc thất bại khi không có đường; đối chiếu topic navigation/UI và
arbiter state, nhấn Stop/đóng UI khi đang chạy, rồi thử lại waypoint và lệnh
trực tiếp cũ. Chưa chạy robot thật hoặc sửa cấu hình Nav2/arbiter trong thay đổi này.

## Phân xử vận tốc và kiểm kê nguồn

| Nguồn/chế độ | Topic trước đây | Topic đầu vào arbiter | Ưu tiên | Timeout |
| --- | --- | --- | --- | --- |
| Nav2 controller + behavior server | `/cmd_vel` | `/cmd_vel_sources/navigation` | 20 | 0,5 s |
| Human following trực tiếp | `/cmd_vel` | `/cmd_vel_sources/following` | 40 | 0,5 s |
| Chuyển động từ giao diện | `/cmd_vel` | `/cmd_vel_sources/ui` | 60 | 0,5 s |
| Docking: rear controller, custom range, opennav | `/cmd_vel` | `/cmd_vel_sources/docking` | 70 | 0,5 s |
| LocalizationWorker xoay định vị | `/cmd_vel` | `/cmd_vel_sources/localization` | 80 | 0,5 s |
| Tay cầm ROS trên mini PC (tùy chọn) | `/cmd_vel` | `/cmd_vel_sources/teleop` | 100 | 0,5 s |

Tay cầm PS2 thực tế của robot nối **trực tiếp STM32**, không đi qua `/joy`
hoặc nhánh teleop ROS trong bảng. Nhánh teleop chỉ áp dụng cho tay cầm USB/ROS
nếu có sử dụng. Firmware tự chọn giữa điều khiển ROS và tay cầm PS2.

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
6. Không có mẫu mới trong 0,5 giây: arbiter thu hồi quyền, phát zero trong
   một đợt ngắn 0,15 giây rồi **ngừng phát `/cmd_vel`**. Khi khởi động hoặc
   đang rảnh, arbiter không phát zero; zero từ nhánh Nav2 đang rảnh cũng không
   làm nó chiếm quyền. Heartbeat trạng thái vẫn chạy 20 Hz.
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
arbiter hoặc không có subscriber đầu ra; khi đó hủy quyền và gửi đợt zero ngắn
nếu trước đó đang phát lệnh.
Kiểm tra ROS graph không thể ngăn một node cấu hình sai tiếp tục ghi trực tiếp;
phải sửa/remap node đó, không nới guard.

### Dừng khẩn cấp và giới hạn phần cứng

Có đầu vào phần mềm `/velocity_arbiter/emergency_stop` (`std_msgs/Bool`): `true`
hủy quyền, chặn mọi nguồn và tiếp tục phát zero khi còn khóa; `false` mở khóa, không khôi phục lệnh
đã hủy. Trạng thái này nằm trong tiến trình, không được lưu qua restart.
Đây là **điểm nối phần mềm**, chưa được nối với nút E-stop vật lý của robot.
Nút Stop trên UI vẫn hủy Nav2 và chuỗi UI; không thay thế E-stop toàn hệ thống.

Nếu **nguồn điều khiển** mất kết nối, arbiter còn sống sẽ dừng sau timeout.
Nếu chính arbiter chết hoặc đường truyền cuối cùng tới STM32 mất, cần watchdog
trong firmware để dừng động cơ; firmware cần tự xử lý. Mã STM32 trong workspace riêng có watchdog 500 ms
trả về điều khiển tay cầm; chưa kiểm chứng động cơ thật trong lần sửa này. Các kiểm thử không chứng nhận khoảng phanh, footprint,
độ phủ lidar, độ trượt hay ưu tiên nút E-stop phần cứng.

## Build và áp dụng

Đã kiểm chứng build cả ba package vào `/tmp/zackon-arbiter-validation`, sau đó
build thành công vào `build/` và `install/` của workspace chính. Đã đối chiếu
launch remap, executable arbiter và cấu hình trong bản cài với mã nguồn.
Lệnh build lại khi có thay đổi:

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

### Kiểm tra chỉ đọc trên robot ngày 04/10/2026

Sau khi build vào workspace chính, kiểm tra ROS domain 0 đang chạy ghi nhận:

| Topic | Publisher thực tế |
| --- | --- |
| `/cmd_vel` | Duy nhất `velocity_arbiter` |
| `/cmd_vel_sources/navigation` | 1 `controller_server` và 5 endpoint `behavior_server` |
| `/cmd_vel_sources/ui` | `robot_ui_motion` |
| `/velocity_arbiter/state` | `velocity_arbiter` |

Heartbeat được nhận với `healthy=true`, `owner=""`, `ui_granted=false`,
`navigation_busy=false`, `emergency_stop=false`; `/cmd_vel` có 1 subscriber.
`ui_granted=false` khi rảnh là bình thường: UI chỉ xin quyền khi nhận lệnh.
Tiến trình Nav2 đang chạy có remap đúng; UI đang phát vào topic riêng.

Đây là ảnh chụp trạng thái tại thời điểm kiểm tra, không bảo đảm trạng thái
sau đó giữ nguyên. Lần kiểm tra này không phát vận tốc, không gọi service/action
điều khiển và không khởi động lại tiến trình. Chưa thử chuyển động trên phần cứng.

### Kiểm tra chỉ đọc ngày 06/10/2026

ROS domain 0 lúc kiểm tra chỉ có UI phát `/cmd_vel_sources/ui`; chưa có
publisher `/cmd_vel`, heartbeat arbiter, Nav2, odometry hoặc hai lidar.
Tiến trình UI và micro-ROS Agent đang chạy; không thấy tiến trình Nav2/arbiter/lidar.
Log launch mới nhất tìm thấy là ngày 04/10; chưa xác định lý do hệ thống chưa
được bật hôm nay. Agent đang chạy không đồng nghĩa STM32 đã kết nối.

Guard nay báo rõ “Bộ điều khiển chuyển động chưa chạy” khi không có publisher
đầu ra, thay vì gộp vào lỗi cấu hình nhiều nguồn. Cần khởi động hệ thống điều
hướng và xác nhận dữ liệu cảm biến trước khi chạy lệnh. Lần kiểm tra này
không khởi động phần cứng hoặc phát lệnh chuyển động.

### Sửa lỗi chiếm tay cầm STM32 ngày 06/10/2026

Người dùng xác nhận tay cầm nối trực tiếp STM32. Đối chiếu mã nguồn
`/home/khoaiuh/stm32_micro_ros_zackon/Core/Src/freertos.c`: `cmdvel_cb` đặt
`autonomous_MODE=1` và gia hạn `last_cmdvel_ms` cho **mọi** Twist, kể cả zero.
`StartTeleopTask` bỏ qua tay cầm khi chế độ này bật và chỉ
trả về manual sau `CMDVEL_TIMEOUT_MS=500` ms không có lệnh ROS.

Bản arbiter trước phát zero liên tục khi rảnh nên không bao giờ cho watchdog
này hết hạn. Đã sửa đầu ra thành ba trạng thái: có lệnh thì phát; vừa kết thúc,
release, mất quyền hoặc timeout thì phát đợt zero 0,15 giây; sau đó im lặng.
Một lệnh mới hợp lệ có thể bắt đầu ngay trong đợt zero. E-stop phần mềm chủ
động là ngoại lệ: giữ zero liên tục đến khi được mở khóa. Sau đợt dừng thông
thường, firmware sẽ trả quyền PS2 khi watchdog hết hạn (500 ms từ mẫu cuối).
Không tự nhận rằng tay cầm PS2 có ưu tiên cao hơn ROS khi ROS đang chạy lệnh;
quy tắc đó do firmware quyết định.

Lỗi odometry được kiểm tra riêng: trong mẫu kiểm tra thực tế, `/odomfromSTM32`
không có publisher với cả subscriber chẩn đoán BEST_EFFORT. Agent systemd đang
nghe UDP 8888 nhưng log kể từ lúc khởi động chỉ có dòng mở cổng, chưa ghi nhận
client/session STM32. Giao diện Ethernet mini PC có IP `192.168.4.150/24`, route
đến STM32 `192.168.4.50` đúng interface; hai lần ping không có phản hồi. Đây là
bằng chứng chưa nhận được dữ liệu từ STM32, **không đủ kết luận hỏng dây/board**.
Kiểm tra tiếp thấy neighbor ARP `192.168.4.50` ở trạng thái `FAILED`, trong
khi cổng mini PC báo `LOWER_UP`/carrier=1. Địa chỉ Agent trong mã firmware là
`192.168.4.150`, trùng IP mini PC; cổng UDP hai phía đều 8888. Log Agent ngày
04/10 từng có session từ `192.168.4.50`, nhưng boot ngày 06/10 chưa có session.
Dịch vụ `stm32_reset.service` đang disabled/inactive; chưa thay đổi trạng thái đó.
Bắt gói chưa thực hiện được vì hệ điều hành yêu cầu mật khẩu sudo, nên không
có kết luận từ packet capture. Cần xác nhận nguồn, đèn link ở phía STM32 và
trạng thái mạng firmware trước khi quy lỗi cho dây/PHY hoặc phần mềm.
Chưa sửa hoặc reset firmware, chưa xác nhận đã khôi phục odometry.

Kiểm thử mới mô hình hóa đúng việc zero cũng gia hạn quyền ROS của STM32;
kiểm tra rảnh, stop burst, timeout, mất graph, lệnh mới và E-stop. Graph giả lập
kiểm tra thêm Nav2 rảnh không phát `/cmd_vel` và hết đợt stop thì im lặng.

Kết quả sau bản sửa này: 53 kiểm thử logic/UI/hồi quy liên quan và 12 kiểm thử
ROS domain 181 đều đạt. Đã build `view_robot_pkg` vào workspace chính và đối
chiếu file trong `install/` khớp mã nguồn. Cần khởi động lại Nav2/arbiter đang
chạy để nạp bản mới. Chưa thử tay cầm/động cơ thật hoặc xác nhận odometry hồi phục.

Kiểm tra tiếp qua ST-Link sau khi người dùng xác nhận đèn link vẫn sáng:
CPU báo `running`, nguồn target khoảng 3,24 V; snapshot CFSR/HFSR đều 0.
Đọc flash và RAM với CPU tiếp tục chạy, không halt/reset/nạp firmware;
đã tắt hook `examine-end` mặc định ghi thanh ghi debug của OpenOCD.
Flash đọc được chứa chuỗi địa chỉ Agent `192.168.4.150`. Các vùng LOAD flash
không khớp các ELF tìm thấy trong project hiện tại (`Debug`, `build-linux/app`)
và `motor_firmware_backup/Debug`. Vì chưa có ELF khớp firmware đang nạp,
không dùng địa chỉ symbol của các bản đó để diễn giải `gnetif`/biến micro-ROS.

Adapter mini PC báo link yes, 10 Mb/s half-duplex qua ethtool; không có thống
kê mở rộng. Chưa có dữ liệu PHY phía STM32 để kết luận lệch speed/duplex.
Cần file ELF đúng của lần nạp hiện tại để đọc tiếp trạng thái mạng bằng symbol;
chưa kết luận firmware mismatch tự nó là nguyên nhân mất odometry.

### Kết quả đọc Ethernet STM32, tiếp ngày 06/10/2026

Project người dùng xác nhận: `/home/khoaiuh/stm32_micro_ros_zackon`.
Không dùng nguyên bảng symbol của ELF không khớp. Đã tìm và đối chiếu riêng
chuỗi lệnh máy của `network_is_ready`, `xTaskGetTickCount` và thân
`HAL_ETH_RxAllocateCallback` trong flash đọc từ mạch; lấy địa chỉ dữ liệu từ
literal của **flash thực tế**. Các kết quả dưới đây là snapshot với CPU chạy,
không halt/reset/ghi RAM/nạp firmware:

- `gnetif` có IP `192.168.4.50`, flags `0x0f` gồm UP/LINK_UP;
  tick RTOS tăng khoảng 1000 trong một giây.
- MAC STM32 đang 10 Mb/s, half-duplex, khớp kết quả adapter mini PC.
  `SCB_CCR=0x00040200`: D-cache không bật tại thời điểm đọc.
- Bốn RX descriptor tại `0x2004c8a0`, bước 40 byte, đều có OWN=1 nhưng
  `DESC2=0` và `BackupAddr0=0`: chưa có địa chỉ buffer nhận hợp lệ.
- Sau phép thử ping/ARP, `ETH_DMASR=0x0260a104`, có fatal bus error và
  receive process stopped. Không diễn giải riêng các bit EBS để khẳng định
  chính xác giao dịch bus nào gây lỗi.
- Thân callback cấp RX buffer khớp duy nhất tại `0x080087a4`; literal pool
  thực tế trỏ tới `0x0803e678`. Descriptor pool cho biết 12 phần tử, kích thước
  `0x620`, con trỏ đầu danh sách trống ở `0x2002b02c`.
  Đọc tại đó được `0`, byte `RxAllocStatus` kế tiếp ở `0x2002b030` bằng `1`
  (`RX_ALLOC_ERROR`). **Pool RX đang cạn**, không chỉ là mất hiển thị topic.

Mã được build theo `tools/sources.mk` là `Core/Src/ethernetif.c`, không phải
bản cùng tên trong `LWIP/Target`. Trong mã hiện tại, cả nhánh reconnect và
force recovery đều gọi Stop/DeInit/Init mà chưa thu hồi RX buffer do driver
giữ. `ETH_DMARxDescListInit` xóa `BackupAddr0`/`DESC2`, làm mất tham chiếu đến
buffer đó. Pool chỉ được khởi tạo lúc đầu. Đây là đường rò buffer cụ thể trong
mã nguồn, phù hợp snapshot cạn pool; chưa chứng minh toàn bộ nhánh recovery
trong flash đang nạp giống hệt nguồn hiện tại.

Ngoài ra, `HAL_ETH_Start_IT` hiện vẫn bật DMA và trả HAL_OK sau khi yêu cầu
build descriptor, kể cả callback không cấp được buffer. Vì vậy netif/link UP
và đèn Ethernet sáng không đảm bảo RX hoạt động.

Bản sửa firmware cần quản lý vòng đời buffer khi recovery: chặn đồng thời
RX/TX, dừng DMA, thu hồi đúng buffer còn thuộc driver (bao gồm gói nhận dở),
giữ nguyên buffer đã giao cho lwIP, rồi mới dựng lại descriptor; chỉ bật RX
khi có buffer hợp lệ. Không gọi lại `LWIP_MEMPOOL_INIT` tùy tiện vì lwIP có
thể vẫn giữ pbuf, dẫn đến cấp trùng hoặc double-free. Cần kiểm thử recovery
lặp nhiều lần, mất/kết nối lại Agent và link, cùng gói đang nhận/truyền.

Lần chẩn đoán này chưa sửa/nạp firmware và chưa khôi phục `/odomfromSTM32`.
Lỗi arbiter phát zero khi rảnh đã sửa ở phía ROS như mục trên; đây là vấn đề
khác với tình trạng RX Ethernet đang cạn buffer trên MCU.

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
