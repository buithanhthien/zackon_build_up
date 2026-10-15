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

Nhân vật tương tác cục bộ và quan sát trạng thái phát TTS để mấp máy miệng;
không gửi lệnh đến AI, robot hoặc ROS.
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


## Mấp máy miệng khi nói — 12/10/2026

`VoiceEngine.playback_changed(bool)` bật ngay trước khi phát từng câu và
luôn tắt khi phát kết thúc, bị dừng hoặc gặp lỗi. Không bật trong lúc tổng
hợp giọng nói; khoảng chờ giữa các câu cũng không làm miệng tiếp tục chạy.
Startup nối signal tới slot Qt của Xoài trên GUI thread. Dùng chung timer
33 ms hiện có, giữ các thao tác chạm/kéo và không vẽ lại khung focus nét đứt.

Miệng mở/khép theo nhịp mô phỏng khi có playback, chưa phân tích biên độ
âm thanh hoặc đồng bộ từng âm vị. Khi kết thúc, trở về biểu cảm hiện tại.
Không thay đổi cách tạo/phát âm thanh hoặc xử lý Dừng.

Kiểm thử liên quan: **20 passed** (mango, startup presentation, voice flow),
bao gồm thay đổi hình miệng, phục hồi biểu cảm, signal khi ẩn/hiện,
phát xong/dừng/lỗi và không mở miệng khi tổng hợp thất bại. Chưa kiểm tra
độ trễ loa và cảm nhận chuyển động trên máy robot thật.

## Trạng thái hội thoại P0 — 14/10/2026

Đã triển khai Đợt 1 của `mango-companion-roadmap.md`. Đợt 2 (idle ngẫu nhiên,
chuyển pose mượt) và Đợt 3 (giảm chuyển động/skin) chưa triển khai.

- `MangoCompanion.set_activity(str)` kiểm tra tập trạng thái `Activity`:
  `idle`, `listening`, `thinking`, `success`, `error`. `speaking` chỉ được
  bật bằng `set_speaking(bool)` từ playback thật. `state` vẫn giữ trạng thái
  tương tác cũ; `display_state` trả về pose đang hiển thị.
- `ChatPanel.activity_changed(str)` báo bắt đầu chờ AI/intent, thu âm, kết
  thúc capture không có transcript, phản hồi rỗng, lỗi AI và hủy. Capture
  kết thúc không xóa trạng thái chờ AI/intent vừa được transcript kích hoạt.
  Lỗi phân loại intent vẫn fallback sang chat theo hành vi hiện có.
- `VoiceEngine.utterance_finished(str)` chỉ bổ sung thông báo kết quả mỗi
  câu: thành công → `success`, hủy/nội dung rỗng sau làm sạch → `idle`, lỗi
  tổng hợp/phát hoặc thiếu bộ tổng hợp → `error`. Mã thoát khác 0 của trình
  phát cũng được ghi nhận. Không đổi hàng đợi, lệnh phát, tổng hợp hay Dừng.
  Slot Qt nhận kết quả trên GUI thread; yêu cầu AI/intent mới đang chờ hoặc
  capture mới đang nghe không bị kết quả TTS cũ thay thế.
- Mắt nhìn trước/nghiêng nhẹ khi nghe; mắt tập trung và miệng phẳng khi chờ;
  nhún rất nhẹ khi nói; cười/vẫy khi thành công; miệng bối rối khi lỗi.
  Miệng vẫn chỉ mấp máy trong playback, kể cả khi đang chạm/kéo.
- Tương tác chạm/kéo/thả có ưu tiên pose. Sự kiện hội thoại không đổi vị trí
  hoặc hủy thao tác. Hoạt động mới thay hoạt động cũ; `success`/`error` hết
  hạn sau 1,1 giây. Hide xóa hoạt động và thao tác, dừng timer; show kiểm tra
  hạn phản hồi đã nhận lúc ẩn trước khi chạy lại timer 33 ms. Playback vẫn
  được theo dõi khi ẩn để hiện lại đúng tình trạng âm thanh.

Đối với motion/navigation, Mango trở về nghỉ khi phân loại xong và chuyển
lệnh sang bộ điều khiển; không báo robot đã thực hiện thành công vì chưa có
nguồn sự kiện kết quả tác vụ tích hợp trong phạm vi này. Không thay callback
robot/ROS/navigation, không thêm dependency hoặc truy cập mạng.

Các file triển khai: `robot_ui/mango_companion.py`,
`robot_ui/chat_panel_widget.py`, `robot_ui/voice_engine.py`,
`robot_ui/startup_layout.py`. Preview cập nhật signal giả lập trong
`tool/preview_startup_ui.py`.

Kiểm thử offscreen bằng lệnh toàn bộ suite ở trên: **260 passed, 14 skipped**;
2 cảnh báo deprecation `aifc`/`audioop`. Bổ sung `tests/test_mango_activity.py`
và mở rộng `robot_ui/test_voice_recording_flow.py`; cập nhật fixture startup
và motion tương ứng. Bao phủ chat/intent/voice, phản hồi rỗng, ưu tiên kéo,
hoạt động mới, cancel/error/success, hide/show, ảnh các pose khác nhau,
queued signal từ thread khác, lỗi tổng hợp, lỗi trình phát, hủy lúc tổng hợp
và playback. Bộ kiểm thử bố cục VI/EN từ 1024×720 vẫn qua.

Chưa nghiệm thu mic/loa, độ trễ và độ mượt trên robot, cảm ứng, AI trực tuyến
hoặc ROS/DDS thật; 14 test bỏ qua vẫn là các ca ROS/DDS opt-in.
