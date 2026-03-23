## 1. Yêu cầu hệ thống

- Ubuntu 22.04
- ROS 2 (Humble / Iron / Jazzy)
- Python ≥ 3.10
- PyQt6
- nav2_map_server
- nav2_amcl
- LiDAR driver (vd: sllidar, rplidar)

---

## 2. Chức năng Docking (Tự động cập cảng)

### 2.1. Cách hoạt động

Khi nhấn nút **Docking** trên giao diện UI, robot sẽ thực hiện các bước sau:

1. **Khởi động docking server** (`opennav_docking`)
2. **Di chuyển đến vị trí staging** (cách dock 0.8m) bằng Nav2
3. **Phát hiện dock** bằng LiDAR - tìm 2 băng phản quang (reflective tape)
4. **Tiếp cận dock** - lùi về phía dock
5. **Hoàn thành** - robot đã vào dock

### 2.2. Cấu hình quan trọng

File cấu hình: `src/lidar_dock_detector/config/docking_params.yaml`

**Thông số phát hiện LiDAR:**
- `i_peak: 43.0` - Ngưỡng cường độ phản xạ của băng phản quang
- `i_valley: 29.0` - Ngưỡng cường độ vùng xung quanh (phải thấp hơn)
- `valley_search_range: 19` - Số beam LiDAR tìm kiếm xung quanh
- `max_detect_range: 3.0` - Khoảng cách phát hiện tối đa (mét)

**Vị trí dock (trong map frame):**
- `home_dock.pose: [-0.40155, 1.34912, 1.57]` - Tọa độ (x, y, yaw) của dock

**Khoảng cách staging:**
- `staging_x_offset: -0.80` - Robot dừng cách dock 0.8m trước khi phát hiện

### 2.3. Debug và Calibration

Nếu robot không phát hiện được dock, sử dụng công cụ debug:

```bash
# Chạy node debug để xem cường độ LiDAR realtime
ros2 run lidar_dock_detector debug_intensity_node
```

Hoặc sử dụng tool Python:
```bash
python3 tool/get_intensities_of_lidar.py
```

Công cụ này sẽ hiển thị:
- Vị trí các peak (băng phản quang)
- Giá trị intensity thực tế
- Giá trị valley xung quanh
- Khoảng cách giữa 2 băng

Dựa vào kết quả, điều chỉnh `i_peak` và `i_valley` trong `docking_params.yaml`.

### 2.4. Mã lỗi Docking (Error Codes)

Khi docking thất bại, hệ thống sẽ trả về mã lỗi:

| Mã lỗi | Tên | Nguyên nhân | Cách khắc phục |
|--------|-----|-------------|----------------|
| 0 | `NONE` | Không có lỗi (thành công) | - |
| 901 | `DOCK_NOT_IN_DB` | Không tìm thấy `dock_id` trong config | Kiểm tra tên dock trong `docking_params.yaml` |
| 902 | `DOCK_NOT_VALID` | Vị trí dock không hợp lệ | Kiểm tra tọa độ `home_dock.pose` |
| 903 | `FAILED_TO_STAGE` | Nav2 không thể đến vị trí staging | Kiểm tra map, đảm bảo đường đi không bị chặn |
| 904 | `FAILED_TO_DETECT_DOCK` | Không phát hiện được băng phản quang sau 10s | Chạy `debug_intensity_node` để điều chỉnh `i_peak`/`i_valley` |
| 905 | `FAILED_TO_CONTROL` | Controller không thể tiếp cận dock (timeout 60s) | Kiểm tra chướng ngại vật, tăng timeout nếu cần |
| 906 | `FAILED_TO_CHARGE` | Không phát hiện sạc sau 5s | Không áp dụng (hệ thống không kiểm tra sạc) |
| 999 | `UNKNOWN` | Lỗi nội bộ không xác định | Kiểm tra log chi tiết |

**Lỗi phổ biến nhất:**
- **904**: Thông số `i_peak`/`i_valley` chưa chính xác → Dùng tool debug để calibrate
- **903**: Đường đi bị chặn hoặc vị trí staging không hợp lệ → Kiểm tra map

### 2.5. Cấu trúc file liên quan

```
src/lidar_dock_detector/
├── src/
│   ├── lidar_intensity_dock.cpp      # Plugin phát hiện dock (ĐANG DÙNG)
│   └── debug_intensity_node.cpp      # Tool debug LiDAR
├── include/lidar_dock_detector/
│   └── lidar_intensity_dock.hpp
├── config/
│   └── docking_params.yaml           # Cấu hình chính
├── launch/
│   └── docking.launch.py             # Launch opennav_docking server
└── plugins.xml                       # Đăng ký plugin với pluginlib

robot_ui/
└── docking_sequence.py               # Script điều khiển docking (được gọi từ UI)

tool/
└── get_intensities_of_lidar.py       # Tool calibrate intensity thresholds
```

---





