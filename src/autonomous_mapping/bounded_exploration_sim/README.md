# Bounded autonomous exploration: Gazebo

Package `bounded_exploration_sim` chạy mô phỏng Gazebo để kiểm thử thuật toán. Luồng dùng LiDAR và Nav2 trên robot thật nằm trong [bounded_exploration_real](../bounded_exploration_real/README.md).

## Hình học và trạng thái

- Hình vuông nhiệm vụ mặc định rộng 10 × 10 m, tâm là `map -> base_link` lúc gọi `start`, cạnh song song trục trước/sau và trái/phải của robot lúc đó. Hai nửa cạnh có thể đổi bằng `half_length_m`, `half_width_m` khi launch.
- Footprint Nav2 là 0,60 × 0,50 m. Mặc định tâm robot phải ở trong vùng ±4,50 m nhờ `footprint_margin_m=0.50`; đường biên ngoài vẫn là ±5 m. Chỉ thay margin sau khi đo lại footprint và sai số định vị.
- Node mission phát `/bounded_exploration/map` từ SLAM `/map`, đánh dấu ô ngoài vùng an toàn là occupied cho `frontier_exploration_ros2`. `/map` gốc vẫn cấp cho Nav2 và map saver.
- Nav2 dùng keepout mask của mission trên **global và local costmap**. Lệnh start đưa mission vào `arming`; explorer chỉ được chạy khi cả hai filter đã subscribe và map/TF/costmap còn mới. Khi đang chạy, mất các tín hiệu này hoặc pose vượt vùng an toàn sẽ dừng mission.
- Explorer chỉ chạy sau service start. Ba bản `/map` khác nhau liên tiếp không có frontier mới tạo sự kiện kết thúc phần frontier. Nếu còn frontier mà không có goal Nav2 nào được chọn trong 45 s, hoặc phiên quá 30 phút, báo `partial` rồi thử về nhà.
- Sau phần frontier, mission mặc định thêm tối đa 6 điểm quan sát trên lưới 2 m (`state=observing`). Chỉ chọn điểm đã biết là trống trên SLAM và global costmap, cách xa đường robot đã đi và nằm trong biên an toàn. Nav2 có thể từ chối điểm không đi tới được; khi đó mission bỏ qua và báo `partial` sau khi thử các điểm khác. Các điểm này giúp robot đi thêm quanh vật cản dù LiDAR đã thấy gần hết vùng từ vị trí đầu.
- Mission dùng Nav2 về pose đã chụp lúc đầu (tối đa 3 lần trong 180 s), kiểm tra lại khoảng cách và hướng thực, sau đó lưu YAML/PGM riêng theo mission ID. Lỗi quay về được ghi `return_failed`; map vẫn được lưu nếu có thể.
- `completed` nghĩa là **không còn frontier có thể khai thác trong vùng theo thuật toán hiện tại**; không bảo đảm mọi ô hình vuông đều được quan sát. Tường cách robot 3 m vẫn được SLAM và costmap cập nhật bình thường; hình vuông không tự co lại.

## Mô phỏng

Mô hình Gazebo trong `urdf/zackon_sim.urdf` được lấy từ cấu hình mô phỏng `zackon_docking/src/view_robot`. `worlds/exploration_room.sdf` là world thử riêng. `worlds/narrow_corridor.sdf` có tường cách vị trí đầu khoảng 3 m bên trái và phải để thử trường hợp phòng hẹp hơn biên 10 × 10 m. `scan_merger.py` dùng logic gộp hai LiDAR từ mô phỏng docking. Nav2 tham số trong `config/nav2_explore.yaml` dựa trên `zackon_docking/src/dock_shape_sim/config/nav2_sim.yaml`, chạy với map SLAM mới và không khởi động AMCL/map server cũ. Các file nằm trọn trong package này để debug độc lập.

Terminal 1:

```bash
cd /home/khoa/zackon_build_up
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to bounded_exploration_sim --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=177 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/tmp/bounded_exploration_ros_logs
ros2 launch bounded_exploration_sim sim.launch.py sim_domain_id:=177 rviz:=true
```

Terminal 2, cùng `ROS_DOMAIN_ID=177` và `source install/setup.bash`. Gọi `start`, rồi chờ log `Sending frontier goal` trước khi trông đợi robot di chuyển. `state=exploring` chỉ xác nhận mission đã bắt đầu.

```bash
ros2 service call /bounded_exploration/start std_srvs/srv/Trigger '{}'
ros2 service call /bounded_exploration/get_status std_srvs/srv/Trigger '{}'
```

Để robot tự quét, đi thêm các điểm quan sát, quay về và lưu map, không gọi `cancel`. Chỉ gọi lệnh sau nếu bạn muốn dừng mission đang chạy:

```bash
ros2 service call /bounded_exploration/cancel std_srvs/srv/Trigger '{}'
```

Trước khi start, có thể đổi kích thước bằng launch args, ví dụ `half_length_m:=4.0 half_width_m:=3.0`. Để tăng số điểm quan sát, thêm `observation_max_points:=10 observation_spacing_m:=1.5` vào lệnh launch; để chỉ dùng frontier như trước, thêm `observation_points_enabled:=false`. Để lưu map ngoài `/tmp`, đổi tham số `map_output_dir` của `/bounded_exploration_mission` trước khi start. RViz hiển thị `/map`, `/explore/frontiers`, `/explore/selected_frontier` và `/bounded_exploration/boundary` (xanh: biên nhiệm vụ; cam: biên tâm robot sau margin; tím: điểm quan sát còn lại; xanh lá: đích quan sát hiện tại). Status có `observation_points_done` và `observation_points_failed`.

Để thử hành lang hẹp, thêm `world_file:=/home/khoa/zackon_build_up/src/autonomous_mapping/bounded_exploration_sim/worlds/narrow_corridor.sdf` vào lệnh `ros2 launch`. Vùng nhiệm vụ vẫn ±5 m từ pose đầu; tường ±3 m sẽ xuất hiện trong `/map` và Nav2 chọn đường chỉ trong phần đi được.

## Kiểm chứng và giới hạn

```bash
PYTHONPATH=src/autonomous_mapping/bounded_exploration_sim /usr/bin/python3 -m unittest discover -s src/autonomous_mapping/bounded_exploration_sim/test -v
source install/setup.bash
ROS_LOG_DIR=/tmp/bounded_exploration_ros_logs ctest --test-dir build/frontier_exploration_ros2 -R test_control_service_and_idle --output-on-failure
```

Đã kiểm chứng build, test hình học, test service explorer, mô phỏng 10 × 10 m có Nav2 đi tới frontier đầu tiên, và ô ngoài biên trên global costmap là lethal (`100`). Bài thử 4 × 4 m đã đi qua chuỗi kết thúc → quay về → lưu map; robot vốn đứng ở home trong bài đó. Bài thử hành lang hẹp với ROI 10 × 10 m đã đi tới frontier, kết thúc khi hết frontier, quay về pose đầu và lưu map. Bài thử mới với `observation_max_points:=2` đã đi thêm hai điểm quan sát, quay về và lưu map. Xem [TEST_REPORT.md](TEST_REPORT.md) để biết kết quả cụ thể. Chưa có kiểm chứng đầy đủ mọi path và quỹ đạo ở vùng 10 × 10 m, sự cố filter trong lúc đang chạy, phần trăm vùng được quan sát, hoặc robot thật. `get_subscription_count()` chứng minh costmap đã subscribe filter nhưng chưa tự chứng minh mọi đường Nav2 luôn nằm trong biên. Để chuyển sang robot thật, cần nối đúng LiDAR, odometry, TF, Nav2 và bộ điều khiển vận tốc hiện có, rồi kiểm chứng đường đi và ranh giới với cấu hình phần cứng đó.
