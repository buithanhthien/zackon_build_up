# Các màn hình mở từ Startup

Các ảnh PNG trong thư mục này được tạo từ widget thật bằng Qt offscreen.
Preview không gọi constructor khởi động ROS/docking/tracking; voice và memory
của tab Hội thoại được thay bằng mock.

Đã đồng bộ Điểm đến, tạo/chọn địa điểm, quản lý/tạo lộ trình, tải/tạo bản đồ,
Ngôn ngữ, Docking và Tracking. Bản đồ popup và thông báo từ Startup cũng dùng
style chung. Nav2, định vị lại và Developer là thao tác khởi chạy/chức năng,
không phải các tab Qt mới; giao diện ứng dụng terminal bên ngoài không thuộc
stylesheet này.

- Palette dùng chung từ `robot_ui/startup_style.py`.
- Các màn hình/hộp thoại dùng `robot_ui/styles.py`.
- Vàng cho thao tác chính, trắng cho thao tác phụ, đỏ cho dừng/xóa.
- Danh sách có trạng thái chọn xanh/vàng; nút tối thiểu 44px và trạng thái
  focus/disabled; bàn phím ảo dùng cùng palette.
- Startup và Tracking dùng chung `refresh_voice_button`: trạng thái nghe,
  xử lý, suy nghĩ và nói nằm trong nút mic.
- Tạo địa điểm có bàn phím ảo; tải bản đồ nhận cả lựa chọn bằng bàn phím.
- Tracking chuyển sang Hội thoại không gọi hàm `focus_input` không tồn tại.

Tạo lại ảnh và kiểm tra kích thước/thao tác cơ bản:

```bash
QT_QPA_PLATFORM=offscreen venv/bin/python tool/preview_startup_screens.py
```

10 màn hình/hộp thoại đã được dựng thử; các cửa sổ và trạng thái mở bàn phím
nằm trong 1024×720. Tracking/Docking được xem trước ở 1024×720. Bộ kiểm thử
hiện có đạt 242 ca, bỏ qua 14 ca ROS/DDS opt-in. Chưa nghiệm thu phần cứng.

Các nhãn thao tác chính được thống nhất tiếng Việt theo giao diện đang dùng;
đợt này không bổ sung bản dịch đầy đủ cho mọi hộp thoại cũ.
