# Startup dashboard — thiết kế và nghiệm thu

Đã triển khai giao diện Startup theo hướng robot operations dashboard: trạng thái thiết bị ở trên, điều khiển giọng nói và Dừng ở bên trái nội dung, hội thoại chiếm vùng rộng bên phải. Menu vận hành tách khỏi Developer.

## Wireframe

```text
┌──────────────────┬─────────────────────────────────────────────────────────┐
│ IUH ROBOT        │ Bảng điều khiển robot              [Bản đồ]   Đồng hồ   │
│                  │ Startup · Giám sát kết nối và vận hành                  │
│ VẬN HÀNH         ├───────────────────┬─────────────────┬───────────────────┤
│ Điểm đến         │ STM32         [i] │ LiDAR trước     │ LiDAR sau         │
│ Về trạm sạc      │ ● Nhãn trạng thái │ ● Nhãn trạng thái│ ● Nhãn trạng thái│
│ Tải bản đồ       ├───────────────────┴─────────────────┴───────────────────┤
│ Bản đồ mới       │ Điều khiển giọng nói │ Hội thoại với Bé Son             │
│ Theo dõi         │ Hướng dẫn thu âm     │                               │
│ Định vị lại      │                      │ Lịch sử có cuộn và xuống dòng  │
│ Nav2             │ [Bắt đầu nghe]       │                               │
│ Ngôn ngữ         │ Trạng thái voice     │                               │
│                  │                      │ [Bàn phím ảo khi mở]           │
│ CÔNG CỤ          │ [■ DỪNG]             │                               │
│ Developer        │ Ý nghĩa thao tác     │ [Nhập tin nhắn…] [⌨] [Gửi]    │
└──────────────────┴──────────────────────┴───────────────────────────────┘
```

## Quy chuẩn

Nguồn style Startup: `robot_ui/startup_style.py`, gồm bảng màu, stylesheet và nội dung tiếng Việt/Anh. Selector giới hạn trong `startup-root`; giữ nguyên `styles.py` và style riêng của dialog/màn hình khác.

- Ba màu chủ đạo theo trang quy định màu của [Sổ tay thương hiệu IUH mới](https://www.senviet.art/wp-content/uploads/2025/11/So-tay-thuong-hieu-IUH-moi-chinh-thuc.pdf): trắng `#ffffff`, IUH Dark Blue `#21409a` (RGB 33, 64, 154), IUH Yellow `#fdb924` (RGB 253, 185, 36). [Trang nhận diện chính thức của IUH](https://iuh.edu.vn/vi/nhan-dien-thuong-hieu.html) xác nhận bộ nhận diện mới dùng xanh–vàng.
- Thẻ và menu trắng trên nền xanh rất nhạt; tiêu đề/icon xanh IUH, mục Tổng quan xanh với vạch vàng; khu voice xanh với chữ trắng, nút nghe và Gửi vàng với chữ xanh. Đang nghe chuyển sang nút trắng/chữ xanh, viền vàng; trạng thái hoạt động dùng xanh. Đỏ `#b42332` dành cho Dừng/mất kết nối, vàng nâu `#865d10` cho nhãn đang kiểm tra để đủ tương phản trên nền trắng.
- Font sans có fallback Noto Sans → DejaVu Sans → sans-serif. Tiêu đề màn hình 24 px, tiêu đề vùng 18 px, nội dung 14–16 px.
- Sidebar 224 px, lề nội dung 24 px, khoảng cách 12–16 px, bo góc 8–12 px. Vùng điều khiển rộng 250–340 px, hội thoại nhận phần rộng còn lại.
- Nút menu, bản đồ, gửi, bàn phím và chẩn đoán có mục tiêu bấm 48 px; micro tối thiểu 112 px, Dừng tối thiểu 80 px. Bàn phím có chiều cao phím tối thiểu 44 px.
- Nhãn trạng thái luôn đi cùng màu. Menu và bản đồ dùng icon nét mảnh vẽ bằng QtGui trong `startup_icons.py` (không cần QtSvg hay font biểu tượng); ký hiệu vuông cho Dừng, `i` cho chẩn đoán, `⌨` cho bàn phím, kèm tên truy cập/tooltip.
- Hover, focus, pressed, checked và disabled có style riêng. Dừng vẫn bấm được khi xử lý voice hoặc chờ hủy điều hướng.

## Phạm vi chức năng

Chỉ thay bốn hàm có sẵn: `init_ui`, `_make_status_card`, `_set_card_status`, `update_language_ui`. Thêm `_refresh_control_presentation` để quan sát trạng thái có sẵn và cập nhật chữ hiển thị; không phát lệnh ROS, không đổi enabled/checked của các nút. Các callback micro, Dừng/TTS, hủy điều hướng, motion, gửi chat, timer kiểm tra thiết bị, Nav2 và shutdown giữ nguyên.

Dòng chờ hủy chỉ hiển thị khi còn goal và đã gửi yêu cầu hủy; không coi việc nhấn Dừng là xác nhận robot đã dừng. Nút micro khi đang thu ghi rõ “Kết thúc thu âm” để phân biệt với Dừng chuyển động.

## Ảnh trước/sau

| Kích thước yêu cầu | Trước | Sau |
| --- | --- | --- |
| 1280×720 | [Trước](before-1280x720.png) | [Sau](after-1280x720.png) |
| 1920×1080 | [Trước](before-1920x1080.png) | [Sau](after-1920x1080.png) |

Bố cục cũ tự tăng cửa sổ 1280×720 thành **1280×827** vì minimum size; ảnh trước phản ánh kích thước thực tế này. Bố cục mới giữ đúng hai kích thước yêu cầu. Các ảnh là render Qt offscreen với ChatPanel giả lập cho phần trình bày, không kết nối ROS hoặc micro.

Các trạng thái minh họa: [Tiếng Việt](after-states-vi-1280x720.png), [English và bàn phím ảo](after-states-en-1280x720.png). Trạng thái thiết bị/voice và nội dung chat trong hai ảnh này là dữ liệu mẫu.

Tạo lại ảnh trước/sau và ảnh minh họa trạng thái:

```bash
QT_QPA_PLATFORM=offscreen python3 tool/preview_startup_ui.py
```

Script đọc bố cục trước từ `HEAD`; nếu commit giao diện mới, cần dùng revision cũ để tái tạo ảnh trước.

## Kết quả kiểm tra tự động

**144 passed, 14 skipped** qua hai lượt:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  tests/test_startup_presentation.py robot_ui/test_voice_recording_flow.py \
  tests/test_virtual_keyboard.py tests/test_motion_commands.py \
  tests/test_motion_navigation.py tests/test_motion_integration.py

source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  tests/test_startup_destinations.py tests/test_new_map_dialog.py \
  tests/test_velocity_arbiter.py tests/test_velocity_arbiter_ros.py
```

14 bài ROS arbiter được bỏ qua vì bộ kiểm tra tích hợp này yêu cầu bật `RUN_ARBITER_ROS_TESTS=1` trong domain cách ly 181; không chạy trên domain robot hiện tại.

Kiểm tra UI mới bao gồm cả tiếng Việt/Anh ở 1024×720, 1280×720, 1920×1080, mở/đóng bàn phím, chat dài, vùng bấm, nhãn ba trạng thái thiết bị và trình bày trạng thái nghe/xử lý/chờ hủy. `git diff --check` và biên dịch Python đã qua. Đối chiếu AST xác nhận tất cả hàm có sẵn ngoài bốn hàm UI nêu trên không đổi.

## Nghiệm thu trên robot thật còn cần thực hiện

Chưa nghiệm thu phần cứng trong phiên này. Người vận hành cần xác nhận:

- STM32 và từng LiDAR: có dữ liệu → hoạt động; ngắt dữ liệu → mất kết nối; kết nối lại → khôi phục; mở chẩn đoán STM32 được ở mọi trạng thái.
- Thu âm một lần, kết thúc thu, xử lý, phát trả lời; gửi bằng Enter/Gửi/bàn phím; không mất lịch sử khi đổi ngôn ngữ.
- Nhấn Dừng khi đang di chuyển, điều hướng, phát tiếng và chờ phản hồi goal; xác nhận chuyển động thực tế dừng theo cơ chế có sẵn.
- Mở/đóng bản đồ, dialog điểm đến, bản đồ mới và đổi ngôn ngữ; kiểm tra thao tác cảm ứng và màn hình 1280×720/1920×1080.

## Tham khảo thiết kế trên Pinterest

Đã tìm mẫu [Dashboard UI Design with Yellow and Blue Colors](https://in.pinterest.com/pin/dashboard-ui-design-with-yellow-and-blue-colors--646336984032178607/) qua tìm kiếm Pinterest. Trang pin chặn truy cập ảnh đầy đủ (403), nên chưa xác nhận trực quan chi tiết của mẫu này. Tham khảo bổ sung bản xem trước truy cập được của [Admin Dashboard Design — Afigo, Samson Efeoghene](https://www.behance.net/gallery/151467693/Admin-Dashboard-Design): thẻ bo góc, nền sáng, phân cấp chữ và điểm nhấn xanh/vàng.

Bản điều chỉnh dùng menu trắng có icon đồng bộ, nhãn Tổng quan riêng, thẻ thiết bị có ô icon và nhãn trạng thái, khu voice xanh IUH và hội thoại trắng thoáng. Không thêm số liệu, biểu đồ hay trạng thái kết nối giả vào ứng dụng; các ảnh trạng thái vẫn là fixture offscreen. Bộ 108 test UI/voice/chat/bàn phím/motion đã chạy lại và qua sau lần điều chỉnh này.

## Sửa thông báo suy nghĩ/xử lý

ChatPanel có timer animation 400 ms; trước đây Startup có timer trình bày 150 ms cùng ghi vào `voice_status_label`, khiến “Bé Son đang suy nghĩ” và “Đang xử lý” luân phiên ghi đè. Startup hiện đọc nhãn gốc và hiển thị qua `voice_status_display` riêng, không sửa nội dung, style hoặc visibility của nhãn nguồn. Khi timer suy nghĩ hoạt động, nhãn hiển thị “Bé Son đang suy nghĩ…”; khi voice engine báo THINKING, hiển thị “Đang xử lý giọng nói…”. Nút micro bị khóa vẫn ghi “Đang xử lý…” theo trạng thái thu âm có sẵn. Các thông báo tiếng Anh cũng phân biệt hai giai đoạn.

Đã kiểm tra thứ tự xen kẽ callback của animation và Startup, bao gồm trước tick suy nghĩ đầu tiên, các tick tiếp theo và lúc ẩn trạng thái: 19 test presentation/voice/keyboard đã qua. Không thay đổi xử lý AI, voice engine hay callback dừng.
