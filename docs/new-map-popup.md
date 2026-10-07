# Bản đồ mới trên Startup

**Bản đồ mới** mở `NewMapUI`, một `QDialog` không modal có Startup làm cha.
Bấm lại sẽ đưa cùng popup ra trước. Startup, ChatPanel và bộ điều khiển giữ
nguyên vòng đời; popup không tạo QApplication, ChatPanel hoặc SYSTEM LOG.

**Bắt đầu lập bản đồ** chạy `MAP_GENERATING.launch.py` trực tiếp trong nhóm
tiến trình riêng, không mở terminal. Nhãn trạng thái xác nhận đã gửi lệnh;
việc tiến trình còn chạy không được coi là bằng chứng SLAM đã sẵn sàng.
Nếu launch kết thúc, popup hiển thị mã thoát và chi tiết lỗi.

**Áp dụng** hoặc Enter trong ô tên chạy `map_saver_cli` bất đồng bộ, lưu vào
`src/view_robot/maps`. Chỉ báo thành công sau mã thoát 0 và có file YAML.
Tên bản đồ không được chứa đường dẫn hoặc ký tự shell. Khi đang lưu,
ô tên và nút Áp dụng bị khóa để tránh gửi nhiều lệnh lưu cùng lúc.

Nút **⌨** cạnh ô tên bật/ẩn bàn phím ảo dùng chung với phần tạo lộ trình.
Phím **Lưu** gọi cùng thao tác với Áp dụng. Bàn phím hỗ trợ chữ Latin không
dấu, Shift, số/ký hiệu, khoảng trắng và xóa lùi; chưa hỗ trợ Telex/VNI.
Ẩn bàn phím giữ nguyên tên đã nhập. Trong lúc lưu, các phím bị khóa cùng ô tên.

**Hủy** dừng nhóm tiến trình lưu và mapping do popup tạo; sau đó có thể bắt
đầu lại. **Quay lại**, Escape và nút X cũng dọn các nhóm này trước khi đóng
popup. Đóng Startup cũng đóng popup và dọn mapping. Không dùng `pkill` theo
tên chung, không dừng RViz/Nav2 hoặc tài nguyên thuộc Startup. Nếu dọn tiến
trình gặp lỗi, hiển thị lỗi và giữ popup để người dùng có thể thử lại.
Một phiên mapping duy nhất được phép chạy giữa các instance trong ứng dụng.

Kiểm thử tự động:

```bash
QT_QPA_PLATFORM=offscreen venv/bin/python -m unittest discover -s tests -p 'test_new_map_dialog.py'
QT_QPA_PLATFORM=offscreen venv/bin/python -m unittest discover -s tests -p 'test_startup_destinations.py'
```

Nghiệm thu trên robot:

1. Bấm Bản đồ mới nhiều lần: chỉ có một popup, Startup vẫn thao tác được.
2. Bắt đầu mapping, kiểm tra SLAM/RViz nhận dữ liệu robot; điều khiển khám phá
   bằng các chức năng trên Startup. Popup không có SYSTEM LOG/Click to Speak/Dừng.
3. Nhập tên, lưu và kiểm tra file YAML cùng ảnh bản đồ; thử tên rỗng và lỗi lưu.
4. Hủy, bắt đầu lại; thử Quay lại, Escape và X khi mapping/đang lưu.
   Xác nhận các tiến trình của phiên mapping đã dừng, tiến trình khác vẫn chạy.
5. Mở lại popup và kiểm tra chat/điều khiển Startup vẫn hoạt động; đóng Startup
   khi mapping để xác nhận nhóm mapping cũng được dọn.
