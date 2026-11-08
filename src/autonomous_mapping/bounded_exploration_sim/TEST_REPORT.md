# Kết quả kiểm chứng mô phỏng

Ngày 2026-11-09, ROS 2 Jazzy. Tất cả lệnh chạy trong workspace `/home/khoa/zackon_build_up`.

| Kiểm tra | Kết quả |
| --- | --- |
| `colcon build --packages-up-to bounded_exploration_sim --symlink-install` | PASS, 3 package |
| `ros2 pkg prefix bounded_exploration_sim` sau `source install/setup.bash` | PASS |
| 4 unit test hình học | PASS |
| `ctest -R test_control_service_and_idle` | PASS, 17 Google tests trong binary |
| Gazebo mặc định, ROI 10 × 10 m | Robot tới frontier đầu; global costmap: tâm `0`, trong biên `0`, ngoài biên an toàn `100`. Hủy thử trước khi quét hết. |
| Gazebo ROI 4 × 4 m | `completed`, `returned_home=true`, YAML/PGM được tạo; robot không rời home trong bài này. |
| Gazebo hành lang rộng khoảng 6 m, ROI vẫn 10 × 10 m | `completed`, `returned_home=true`; robot tới frontier rồi đi về, `max_local_x_m=3.124`, `max_local_y_m=1.609`. Map YAML/PGM được tạo. |
| Gazebo hành lang, `observation_max_points:=2` | Sau frontier, Nav2 tới hai điểm quan sát `(3,-1)` và `(1,-1)`, rồi robot quay về home và lưu map. `observation_points_done=2`, `observation_points_failed=0`, `completed`, `returned_home=true`. |

Ở bài hành lang, lần về đầu tiên đạt goal của Nav2 nhưng chưa qua kiểm tra pose chặt hơn của mission. Mission thử lại và lần thứ hai qua kiểm tra pose. Log: `/tmp/bounded_corridor_fixed_launch.log`; map: `/tmp/bounded_exploration_maps/exploration_b1b62de9957d4baebd731a0767f626db.yaml` và `.pgm`. Các file `/tmp` chỉ là artifact thử nghiệm, có thể bị hệ thống xóa.

Bài có điểm quan sát bổ sung ghi log tại `/tmp/bounded_observation_launch.log`, map tại `/tmp/bounded_exploration_maps/exploration_bd879fde2bd14a3d82abdf0a826461af.yaml` và `.pgm`.

Kết quả này xác nhận tường cách vị trí đầu khoảng 3 m sẽ xuất hiện trên SLAM và robot vẫn hoàn tất phần có thể đi trong ROI ±5 m. Vùng nhiệm vụ giữ nguyên hình vuông; thuật toán không tự co hình vuông theo tường. `completed` nghĩa là không còn frontier phù hợp theo bộ tìm frontier hiện tại, không phải chứng nhận 100% diện tích đã được quan sát.

Chưa kiểm tra mọi path và mọi điểm quỹ đạo trong nhiều world, gián đoạn filter khi đang chạy, hoặc robot thật. Các thử nghiệm này cần trước khi dùng luồng với phần cứng.
