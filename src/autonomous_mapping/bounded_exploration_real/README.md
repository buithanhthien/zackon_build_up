# Bounded autonomous exploration: robot thật

Package `bounded_exploration_real` dùng thuật toán frontier và mission đã thử trong [bản mô phỏng](../bounded_exploration_sim/README.md), nhưng chạy với phần cứng trong `view_robot_pkg`. Package này không phụ thuộc Gazebo hay `bounded_exploration_sim`. Hai bản có source `mission.py` và `geometry.py` riêng để có thể debug và điều chỉnh cho robot thật độc lập.

## Luồng chạy

`real.launch.py` khởi động `MAP_GENERATING.launch.py` của `view_robot_pkg` (driver, TF, EKF, `/merged`, velocity arbiter, SLAM trên `/map`), Nav2 cho phiên quét, frontier explorer và mission. Nav2 dùng các tham số phần cứng từ `view_robot/config/nav2_params.yaml`; launch tạo một bản YAML tạm trong `/tmp`, đổi keepout filter của **cả local lẫn global costmap** sang `/bounded_exploration/filter_info`. File gốc trong `view_robot` không bị sửa. Lệnh vận tốc đi qua controller/behavior → velocity smoother → collision monitor → `/cmd_vel_sources/navigation` → velocity arbiter → `/cmd_vel`.

Mission tạo vùng chữ nhật theo pose lúc `start`, mặc định ±5 m theo trục trước/sau và trái/phải, với margin tâm robot 0,50 m. Nó chỉ bắt đầu explorer khi `/map`, TF `map → base_link`, hai costmap/filter, `/merged` và trạng thái velocity arbiter đều còn mới. Mất một tín hiệu trong lúc chạy khiến mission dừng và phát emergency stop. Sau khi hết frontier, robot có thể đi thêm tối đa 6 điểm quan sát, quay về pose ban đầu và lưu map vào `~/zackon_maps/bounded_exploration`. Nếu emergency stop đã bật, cần kiểm tra nguyên nhân rồi giải phóng bằng quy trình vận hành của robot trước phiên tiếp theo.

## Chạy

Trong workspace này:

```bash
cd /home/khoa/zackon_build_up
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to bounded_exploration_real --symlink-install
source install/setup.bash
ros2 launch bounded_exploration_real real.launch.py
```

Launch không tự gọi `start`. Khi driver, LiDAR, SLAM và Nav2 đã lên, kiểm tra RViz, vùng trống trước robot, E-stop vật lý và khả năng điều khiển thủ công. Sau đó tại terminal khác cùng `ROS_DOMAIN_ID` và cùng overlay:

```bash
ros2 service call /bounded_exploration/start std_srvs/srv/Trigger '{}'
ros2 service call /bounded_exploration/get_status std_srvs/srv/Trigger '{}'
```

Chỉ gọi `/bounded_exploration/cancel` khi cần dừng mission. `state=arming` nghĩa là vẫn chờ điều kiện; xem `detail` để biết thiếu LiDAR, arbiter, filter hay TF. `state=completed` hoặc `partial` có `map_path` cho file `.yaml`/`.pgm`. `return_failed` nghĩa là đã thử về vị trí đầu nhưng không xác nhận được; cần xử lý robot tại chỗ.

Để dùng driver và SLAM đã chạy từ một terminal khác, khởi động launch với `start_hardware:=false start_slam:=false`; launch vẫn tạo Nav2 cho phiên quét. Không chạy đồng thời Nav2 tự hành cũ hoặc AMCL/map server cũ: một phiên chỉ có một Nav2 và một nguồn TF `map → odom`. Nếu đã có Nav2 riêng, `start_nav2:=false` chỉ phù hợp khi Nav2 đó **đã** dùng keepout filter động `/bounded_exploration/filter_info` trên cả hai costmap; nếu không, mission sẽ ở `arming`.

Các tham số có thể đổi ngay khi launch: `half_length_m`, `half_width_m`, `observation_points_enabled`, `observation_spacing_m`, `observation_max_points`, `map_output_dir`. Ví dụ `half_length_m:=4.0 half_width_m:=3.0 observation_max_points:=10`. Vùng không tự thu nhỏ khi gặp tường; SLAM và costmap đánh dấu tường, robot chỉ đi trong phần có đường đi. `completed` chỉ xác nhận thuật toán không tìm thấy frontier tiếp, không chứng minh đã quan sát 100% diện tích.

Package mới chỉ được build và kiểm tra cấu hình tĩnh; cần thử tiến dần trên robot thật với người giám sát và E-stop trong tầm tay trước khi dùng cho khu vực lớn.
