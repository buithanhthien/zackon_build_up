# Rà soát ổn định startup, motion và chat — 07/10/2026

Rà soát trên working tree có sẵn thay đổi UI/UX, không hoàn nguyên thiết kế
startup của người dùng. Không khởi động robot, gửi goal Nav2 hay gọi API AI.

## Các lỗi đã sửa

| Phát hiện | Hậu quả | Sửa |
| --- | --- | --- |
| `RobotUI.__init__` tạo hai `RosMotionController`, nối signal hai lần | Hai node `robot_ui_motion`, mất tham chiếu controller đầu, lệnh và log có thể nhận hai lần | Chỉ khởi tạo/nối một lần; bỏ import, khai báo signal, heartbeat và parse motion lặp |
| `LocalizationWorker.run` tạo hai publisher cùng topic | Tạo thừa endpoint điều khiển | Giữ một publisher |
| Policy trả lời thiếu cấm emoji; `_on_response` nhận nguyên văn | Emoji xuất hiện trong lịch sử, log, UI và đầu vào TTS | Thêm policy plain text và bộ lọc emoji dùng chung trước các đầu ra; không gửi TTS nếu lọc thành rỗng |
| Subscription trạng thái cảm biến dùng reliable mặc định | Không nhận được dữ liệu từ publisher best-effort, UI có thể báo mất kết nối sai | Dùng `qos_profile_sensor_data` cho odometry và hai lidar |
| Đóng motion trước khi popup mapping có thể từ chối đóng | Cửa sổ còn mở nhưng motion đã bị hủy | Kiểm tra popup trước khi hủy motion |
| Docking/tracking được mở trước khi kiểm tra kết quả `close()` | Có thể mở chế độ mới trong khi cửa sổ cũ vẫn chờ hủy navigation | Chỉ mở tiến trình sau khi đóng thành công |
| Dừng không yêu cầu luồng định vị ngừng xoay | Luồng localization có thể tiếp tục phát vận tốc | Dừng localization cùng motion/navigation; chờ luồng kết thúc trước khi hủy tài nguyên cửa sổ |

Emoji ở ví dụ log là nội dung do AI sinh. Widget chèn nội dung bằng
`QTextCursor.insertText`, không tự chuyển một câu chào thành icon.
Bộ lọc giữ dấu tiếng Việt, số, xuống dòng và ký hiệu toán thông dụng;
không phải bộ chuyển đổi Markdown tổng quát. Quy tắc không dùng emoticon
ASCII nằm ở prompt; bộ lọc tập trung vào emoji Unicode.

## Kiểm thử

- `tests`: **233 passed, 14 skipped**. Các ca bỏ qua là ROS/DDS opt-in.
- `robot_ui/test_voice_recording_flow.py`: **9 passed**.
- Bao gồm viewport và bàn phím cảm ứng ở 1024×720, 1280×720, 1920×1080,
  tiếng Việt/Anh; motion parser, arbiter, vòng đời goal/cancel; waypoint,
  mapping popup; chat routing và nguồn web.
- Thêm 8 ca hồi quy trong `tests/test_startup_stability.py`: constructor chỉ
  sở hữu một controller/nối một lần; cùng nội dung sạch cho history/UI/TTS;
  emoji ghép; câu rỗng; close veto; chuyển chế độ; dừng/chờ localization.
- `git diff --check` sạch. Phân tích AST các file Python trong `robot_ui`
  không còn định nghĩa trùng hoặc câu lệnh liền kề trùng trong mã chương trình.

Venv thiếu pytest, nên dùng bản pytest đã có trên máy bằng cách append đường
dẫn hệ thống sau site-packages của venv (không cài thêm hay đổi dependency):

```bash
QT_QPA_PLATFORM=offscreen venv/bin/python -c 'import sys; sys.path.append("/usr/lib/python3/dist-packages"); import pytest; raise SystemExit(pytest.main(["tests", "robot_ui/test_voice_recording_flow.py", "-q"]))'
```

## Phần chưa được nghiệm thu

Chưa kiểm tra DDS, cảm biến, phanh/dừng vật lý, âm thanh thật hoặc phản hồi AI
trực tuyến. Kiểm thử mock không xác nhận những phần này.

Thiết kế định vị đứng yên và kiểm chứng pose/AMCL trong
[startup-localization-handoff.md](startup-localization-handoff.md) vẫn là
hạng mục bàn giao, chưa triển khai trong đợt sửa ổn định này. Mã hiện tại còn
đánh giá hội tụ bằng covariance và timer, chưa xác nhận đầy đủ map identity,
độ mới pose và kết quả service. Không coi việc UI khởi động thành công là
bằng chứng định vị đúng.

Sau khi đóng hoàn toàn tiến trình UI cũ và chạy lại, cần kiểm tra trên máy
robot: chỉ một node motion cho một cửa sổ startup; trạng thái lidar/STM32;
câu chào không emoji; Dừng khi định vị; từ chối đóng mapping không làm mất
motion; từ chối đóng startup không mở thêm docking/tracking.
