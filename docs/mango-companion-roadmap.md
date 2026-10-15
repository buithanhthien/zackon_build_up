# Roadmap: làm Mango sinh động hơn

Tài liệu handoff để triển khai từng đợt. Mục tiêu là giúp Mango thể hiện rõ
trạng thái của trợ lý và có chuyển động tự nhiên hơn, nhưng vẫn giữ giao diện
gọn, không gây xao nhãng và không ảnh hưởng điều khiển robot.

## 1. Dữ liệu và hiện trạng đã đối chiếu

### Trong dự án hiện tại

- `robot_ui/mango_companion.py` vẽ nhân vật vector bằng `QPainter`, chạy timer
  Qt 33 ms, dùng đồng hồ monotonic và không cần mạng, audio riêng hay thư viện
  hình động. Đã có `idle`, `touched`, `happy`, `dragging`, `released`; click,
  Space/Enter và kéo/thả đã được xử lý.
- TTS phát `VoiceEngine.playback_changed(bool)` khi bắt đầu/kết thúc playback.
  `startup_layout.py` đã kết nối signal này với `MangoCompanion.set_speaking`.
  Miệng hiện mấp máy khi audio thực sự phát, không trong lúc tổng hợp.
- `VoiceState` trong `robot_ui/voice_engine.py` có `LISTENING`, `THINKING`,
  `SPEAKING`; `ChatPanel._on_voice_state_changed` cập nhật trạng thái UI từ
  các giá trị đó. Hỏi AI bằng chat văn bản cũng có trạng thái chờ qua
  `_typing_timer`, nhưng không nhất thiết đi qua `VoiceState`.
- Lỗi hỏi AI được xử lý ở `ChatPanel._on_error`; câu trả lời được nhận tại
  `_on_response`. Hiện Mango chưa nhận sự kiện hoạt động/lỗi của hai luồng này.
- Bố cục chia khu vực Mango/chat; tài liệu hiện tại đặt yêu cầu tương thích
  cửa sổ từ 1024×720, hỗ trợ tiếng Việt/Anh và không làm đổi callback mic,
  Dừng, chat, ROS hoặc navigation.

### Ý tưởng tham khảo

- [Petdex](https://github.com/crafter-station/petdex) dùng các trạng thái
  animation đặt tên rõ ràng theo hoạt động, gồm `idle`, `waiting`, `running`,
  `review`, `failed`, `waving`, `jumping`; renderer còn có nhịp chớp mắt idle
  không đều. Ý tưởng phù hợp để học là **ánh xạ sự kiện thật sang trạng thái
  nhân vật**, không phải nhập nguyên spritesheet/cơ chế coding-agent.
- [OpenAnima](https://github.com/Ertugrulmutlu/OpenAnima) tập trung vào
  asset overlay có thể kéo, đổi kích thước, tốc độ và lưu trạng thái. Các khả
  năng quản lý overlay ít phù hợp hơn vì Mango đang là widget nằm trong cửa sổ
  ứng dụng robot, không phải desktop pet độc lập.

## 2. Danh sách tính năng đề xuất

| Ưu tiên | Tính năng | Sự kiện kích hoạt | Phản hồi gợi ý | Phạm vi |
| --- | --- | --- | --- | --- |
| P0 | Trạng thái đang nghe | Voice chuyển sang `LISTENING` | Hướng mắt về phía trước, nghiêng đầu nhẹ; không chạy liên tục mạnh | MVP |
| P0 | Trạng thái đang xử lý | Voice `THINKING` hoặc chat/intent bắt đầu chờ | Chớp mắt/chuyển động lá chậm, biểu cảm tập trung | MVP |
| P0 | Trạng thái đang nói | TTS playback hiện có | Giữ mấp máy miệng; thêm nhún/nhấn nhịp rất nhẹ | MVP |
| P0 | Phản hồi hoàn tất | Câu trả lời hoàn tất phát hoặc tác vụ hoàn tất thành công | Nụ cười/vẫy tay ngắn, rồi trở về trạng thái hoạt động hoặc nghỉ | MVP |
| P0 | Phản hồi lỗi | `_on_error` hoặc lỗi tác vụ có nguồn sự kiện rõ ràng | Biểu cảm bối rối nhẹ trong thời gian ngắn, sau đó phục hồi; không dùng animation gây báo động | MVP |
| P1 | Idle có biến thể | Khi không có tương tác/hoạt động | Nhịp thở, chớp mắt có khoảng nghỉ không đều, lá đung đưa nhỏ | Polish |
| P1 | Phản ứng khi được chạm | Click/tap đã có | Thêm 1–2 phản ứng ngắn ngẫu nhiên có kiểm soát (vẫy/nhún), không phát âm thanh | Polish |
| P1 | Chuyển trạng thái mượt | Mọi lần đổi trạng thái | Nội suy góc/độ cao/biểu cảm trong khoảng ngắn thay vì đổi pose đột ngột | Polish |
| P2 | Tùy chọn giảm chuyển động | Cài đặt UI, nếu được yêu cầu | Giảm hoặc tắt chuyển động trang trí nhưng giữ biểu cảm thiết yếu | Tùy chọn |
| P2 | Gói hình/skin thay thế | Chỉ khi có nhu cầu đổi phong cách | Đánh giá sprite/APNG riêng; không thay renderer trong đợt đầu | Tùy chọn |

## 3. Handoff triển khai theo đợt

### Đợt 1 — Trạng thái hội thoại có nguồn sự kiện rõ (P0)

1. Mở rộng API trạng thái của `MangoCompanion` để nhận trạng thái hoạt động
   hữu hạn, có kiểm soát (ví dụ `idle`, `listening`, `thinking`, `speaking`,
   `success`, `error`); không rải chuỗi trạng thái tuỳ ý trong các hàm vẽ.
2. Giữ `set_speaking(bool)` tương thích hoặc chuyển đổi có kiểm soát; hiện
   `playback_changed` là nguồn chuẩn cho việc mấp máy miệng. Không suy ra
   playback từ việc TTS bắt đầu tổng hợp.
3. Cho Mango nhận sự kiện từ `ChatPanel` qua signal/slot Qt hoặc API tích hợp
   hiện có. Cần bao phủ chat văn bản lẫn giọng nói:
   - bắt đầu chờ AI/intent → `thinking`;
   - voice capture → `listening`;
   - playback thực sự → `speaking`;
   - lỗi → `error` tạm thời;
   - hoàn tất thành công → `success` ngắn, rồi `idle`.
4. Tránh nối Mango trực tiếp vào chi tiết worker/thread hoặc phân tích nội
   dung hội thoại. Emit sự kiện ở ranh giới UI/thread-safe hiện có; cập nhật
   widget trên GUI thread.
5. Định nghĩa thứ tự ưu tiên trạng thái để không xung đột:
   - tương tác trực tiếp (`touched`/`dragging`/`released`) không bị sự kiện
     chat làm mất thao tác kéo;
   - `speaking` điều khiển miệng trong khi audio chạy;
   - trạng thái hoạt động mới hơn thay thế trạng thái hoạt động cũ;
   - `success`/`error` có thời hạn và không kẹt sau cancel, hide hoặc lỗi.
6. Vẽ các khác biệt nhỏ bằng QPainter hiện có: mắt/miệng, nghiêng thân, vị trí
   lá/tay. Dùng cùng art-space 240×260 và giữ hình trong vùng vẽ.

### Đợt 2 — Idle và chuyển động tự nhiên (P1)

1. Thêm biến thể idle chậm, không đồng bộ cứng mỗi frame với một chu kỳ dễ
   nhận thấy; giữ chuyển động nhẹ để không cạnh tranh với chat.
2. Chuyển pose giữa trạng thái bằng nội suy theo thời gian monotonic; không
   dùng `sleep`, worker mới, timer dày hơn hoặc tải tài nguyên mạng.
3. Thêm một vài phản ứng chạm có kiểm soát; giữ nguyên hit-test, vùng kéo,
   click chuột phải và hành vi Space/Enter.
4. Nếu timer tiếp tục chạy 33 ms, chỉ tính/vẽ lại hiệu ứng đang cần; đảm bảo
   timer dừng khi widget ẩn như hiện tại.

### Đợt 3 — Tùy chọn và nghiệm thu (P2)

1. Chỉ thêm giảm chuyển động hoặc thay asset sau khi được yêu cầu riêng và
   thống nhất cách lưu/cấu hình.
2. Ưu tiên vector hiện tại. Không tải hoặc sao chép pet art/spritesheet từ
   Petdex; nếu chọn asset bên ngoài, cần xác nhận giấy phép và nguồn.

## 4. Tiêu chí nghiệm thu

- Mango phản ánh đúng các trạng thái `listening`, `thinking`, `speaking`,
  `success`, `error`; cả chat văn bản và luồng voice được bao phủ.
- Hủy phát, lỗi tổng hợp/phát, kết thúc capture, câu trả lời rỗng, lỗi AI,
  đóng/ẩn/mở lại đều không để Mango mắc ở một trạng thái hoạt động.
- Khi `speaking` kết thúc, miệng đóng và Mango trở về trạng thái hoạt động
  còn hợp lệ hoặc `idle`; không nhầm thời gian tổng hợp thành thời gian nói.
- Kéo thả và chạm hiện tại vẫn hoạt động; sự kiện hoạt động không làm lệch vị
  trí, không kích hoạt mic/Dừng/gửi chat và không thay nội dung chat.
- Không thay đổi hành vi ROS, navigation, voice synthesis/playback hoặc xử lý
  hội thoại; không thêm dependency hay gọi mạng.
- Kiểm thử Qt/offscreen bao gồm ánh xạ sự kiện → trạng thái, ưu tiên/cancel,
  timer khi hide/show, ảnh thay đổi theo animation và tương thích bố cục hiện
  có ở tiếng Việt/Anh, 1024×720 trở lên.
- Cập nhật `docs/virtual-mango-companion.md` sau khi triển khai và chạy bộ
  test Mango/startup/voice liên quan.

## 5. Ngoài phạm vi đợt đầu

- Không tích hợp Petdex CLI, API, catalog, agent hooks hoặc định dạng
  spritesheet của họ.
- Không biến Mango thành cửa sổ desktop overlay độc lập, không cho kéo ra
  ngoài khu vực widget.
- Không thêm đói/ngủ/thân mật, chỉ số tâm trạng, âm thanh/hiệu ứng gây xao
  nhãng, hay suy luận cảm xúc người dùng.
- Không gửi lệnh robot/ROS khi chạm Mango; companion chỉ phản ánh UI.

## Prompt có thể chuyển cho AI triển khai

> Hãy đọc `docs/mango-companion-roadmap.md`,
> `docs/virtual-mango-companion.md`, `robot_ui/mango_companion.py`,
> `robot_ui/chat_panel_widget.py`, `robot_ui/voice_engine.py`,
> `robot_ui/startup_layout.py` và các test Mango/startup/voice liên quan.
> Triển khai **Đợt 1 (P0)** theo roadmap, giữ tương thích hành vi click/kéo,
> playback signal, giao diện tiếng Việt/Anh và các callback hiện có. Dùng
> signal/slot Qt thread-safe để báo trạng thái chat/voice; không gọi mạng,
> không thêm dependency, không thay đổi ROS/navigation/TTS. Thêm test cho
> ánh xạ trạng thái, ưu tiên tương tác, cancel/error/empty reply, hide/show và
> playback; cập nhật tài liệu MVP. Chạy các test liên quan và báo rõ file đổi,
> test đã chạy và phần nào chưa thể nghiệm thu trên robot thật.
