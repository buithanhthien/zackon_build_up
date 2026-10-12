# Nhân vật xoài — MVP

Branch: `feature/virtual-mango-companion`, tạo từ `feature/test_chat_AI`.

## Hành vi đã chọn trước khi tạo hình

| Trạng thái | Thao tác / thời gian | Phản hồi |
| --- | --- | --- |
| Nghỉ | Tự động khi hiển thị | Nhún nhẹ, tay đung đưa, chớp mắt định kỳ |
| Được chạm | Nhấn chuột trái trên nhân vật | Nheo mắt, nghiêng người |
| Vui | Thả sau click; Space/Enter khi có focus | Cười, vẫy tay khoảng một giây rồi nghỉ |
| Bị kéo | Giữ trái và di chuyển qua ngưỡng kéo của Qt | Đi theo chuột, giới hạn trong sân riêng |
| Thả | Thả sau kéo | Bật nhẹ tại vị trí mới rồi nghỉ |

Nhân vật ở trái vùng nội dung, chat ở phải; menu và trạng thái thiết bị giữ
vị trí hiện có. Nút nghe và Dừng nằm dưới nhân vật, giữ callback hiện có.
Sân tối thiểu cao 110 px; hình dùng hệ tọa độ 240×260, scale đồng nhất, tối đa
1.25 lần, chừa biên cho chuyển động. Vị trí được lưu theo tỷ lệ trong sân nên
resize không đưa hình ra ngoài. Kích thước cửa sổ hỗ trợ từ 1024×720, kể cả
khi mở bàn phím. Không hỗ trợ kéo sang chat. Khi ẩn/đóng, dừng timer và hủy
thao tác kéo; hiện lại bắt đầu từ trạng thái nghỉ.

## Tạo hình và phạm vi

Thiết kế nguyên bản bằng QPainter trong `robot_ui/mango_companion.py`: thân
xoài bất đối xứng vàng cam, lá xanh, tay chân và nét mặt vẽ bằng đường cong.
Các bộ phận dùng lại ở mọi trạng thái, không có bitmap, font icon, âm thanh
hay nội dung tải từ mạng. Không cần thư viện mới. Timer GUI 33 ms, tính
chuyển động theo đồng hồ monotonic, không có worker hoặc sleep.

Nhân vật chỉ tương tác cục bộ, không nối với AI, TTS, robot hoặc ROS.
Chạm màn hình dùng sự kiện chuột tổng hợp của Qt; cần nghiệm thu cảm ứng
trên thiết bị thật. Không có âm thanh hoặc trạng thái đói/ngủ trong MVP.

## Chạy và xem trước

Từ thư mục gốc, trên máy có desktop và PyQt6:

```bash
venv/bin/python tool/preview_mango_companion.py
```

Preview dùng các hàm bố cục Startup thật với ChatPanel giả lập; có thể chơi
với Xoài, resize và bật bàn phím. Micro, gửi chat và nút vận hành trong
preview không gọi dịch vụ thật. Ứng dụng robot vẫn khởi chạy theo quy trình
hiện có (`./start_beson.sh`); nhân vật được tạo trong `RobotUI.init_ui`.

Tạo ảnh offline:

```bash
venv/bin/python tool/preview_mango_companion.py --output docs/startup-ui/mango-1280x720.png
venv/bin/python tool/preview_mango_companion.py --output docs/startup-ui/mango-keyboard-1024x720.png --width 1024 --keyboard
```

- [1280×720](startup-ui/mango-1280x720.png)
- [1024×720, mở bàn phím](startup-ui/mango-keyboard-1024x720.png)

## Kết quả kiểm thử — 07/10/2026

```bash
QT_QPA_PLATFORM=offscreen venv/bin/python -c 'import sys; sys.path.append("/usr/lib/python3/dist-packages"); import pytest; raise SystemExit(pytest.main(["tests", "robot_ui/test_voice_recording_flow.py", "-q"]))'
```

**246 passed, 14 skipped**, hai cảnh báo deprecation `aifc`/`audioop` từ
SpeechRecognition. Các ca bỏ qua là ROS/DDS opt-in. Venv chưa có pytest nên
lệnh dùng pytest có sẵn của hệ thống, không cài thêm dependency.

- Kiểm thử chuột Qt thật: chạm, vui, kéo ra ngoài sân, thả và trở về nghỉ;
  Space; bấm vùng trống/phím chuột phải không kích hoạt nhân vật.
- Resize giữ đúng tỷ lệ hình và giới hạn vị trí; animation thay đổi ảnh;
  ẩn/hiện/đóng dừng và khởi động timer, hủy thao tác đang giữ.
- Startup tiếng Việt/Anh, 1024×720, 1280×720, 1920×1080, chat dài và mở/đóng
  bàn phím: nhân vật luôn bên trái chat, sân ≥110 px, lịch sử ≥80 px,
  các nút còn trong cửa sổ và cao ≥44 px.
- Bấm nhân vật không phát signal click của nghe, Dừng hoặc gửi, không đổi
  nội dung đang nhập hay lịch sử. Bộ hồi quy chat/voice/motion đã qua.
- Đã xem hai ảnh Qt offscreen ở trên; `git diff --check` sạch.

Phần mã Startup chỉ đổi import và hai hàm trình bày `init_ui`,
`update_language_ui`; nút nghe/Dừng vẫn dùng đối tượng và kết nối của
ChatPanel hiện có. Không sửa `chat_panel_widget.py`, voice engine, ROS,
navigation, motion hoặc shutdown. Nút nghe giảm minimum từ 112 xuống 56 px,
Dừng từ 80 xuống 48 px; vùng trái rộng 250–480 px, chia đều phần nội dung
với chat khi đủ chỗ, chat nhận phần dư khi vùng trái đạt trần.

Chưa nghiệm thu âm thanh, AI trực tuyến, ROS/DDS, robot và màn hình cảm ứng
thật. Kiểm thử offscreen không đánh giá tốc độ khung hình trên máy robot;
cần thử kéo/thả, thu/phát giọng nói và đóng/mở trên thiết bị đích.
