# Hướng dẫn Khởi chạy micro-ROS và Joystick

Tài liệu này hướng dẫn các bước để khởi chạy hệ thống điều khiển robot. Quy trình này yêu cầu mở **4 cửa sổ terminal riêng biệt** để chạy song song các tiến trình sau:

1.  **micro-ROS Agent:** Cầu nối giữa robot (ESP32) và ROS 2
2.  **Joy Node:** Trình điều khiển (driver) đọc dữ liệu từ tay cầm
3.  **Teleop Node:** Node của bạn (`my_joy_teleop`) để chuyển đổi dữ liệu tay cầm thành lệnh
4.  **Debug:** Một terminal để kiểm tra kết quả

## 1. Chuẩn bị

Một số điều cần chuẩn bị trước khi bắt đầu:
* Gắn ESP32 có chứa code điều khiển
* Cắm joystick (tay cầm) vào mini pc
* Tạo thư mục workspace trong home
> cd ~
> mkdir workspace
* Clone repo trên github vào workspace vừa tạo
> cd ~/workspace
> git clone -b feat/update-files https://github.com/khoihoanglsr/MangoMobileRobot.git
* Biên dịch (build) thành công MangoMobileRobot/src (chứa `my_joy_teleop`).
> cd ~/workspace/MangoMobileRobot 
> colcon build
---

## 2. Các bước thực thi

### Terminal 1: Khởi chạy micro-ROS Agent

Terminal này chịu trách nhiệm kết nối với robot

**1. Tìm cổng serial của robot:**
Chạy lệnh sau để tìm đường dẫn chính xác đến phần cứng:

> ls /dev/serial/by-id/*

Bạn sẽ thấy một kết quả, ví dụ: 
> /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

2. Sao chép đường dẫn và khởi chạy agent:

THAY THẾ --dev /dev/serial/by-id/... bằng đường dẫn bạn vừa tìm thấy
    > ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

Lưu ý: Hãy để terminal này chạy. Nếu nó kết nối thành công, bạn sẽ thấy thông báo "Session established".

### Terminal 2: Khởi chạy Trình điều khiển Joystick
Terminal này đọc tín hiệu thô từ tay cầm.

1. Kiểm tra xem joystick đã kết nối chưa
> ls /dev/input/js*

Nếu xuất hiện /dev/input/js0 hoặc tương tự thì là đã kết nối thành công

2. Cấp quyền cho joystick: Lệnh này chỉ cần chạy một lần sau khi cắm tay cầm.
> sudo chmod a+r /dev/input/js0

3. Chạy node joy_linux:
> ros2 run joy_linux joy_linux_node

Lưu ý: Hãy để terminal này chạy. Nó đang liên tục phát (publish) dữ liệu tay cầm lên topic /joy.

### Terminal 3: Khởi chạy Node Teleop
Terminal này chạy code my_joy_teleop để chuyển đổi topic /joy thành topic /cmd_vel.
ros2 run my_joy_teleop my_teleop_node 
1. Source workspace:
Đảm bảo ở đúng thư mục workspace
> cd ~/workspace/MangoMobileRobot 
> source install/setup.zsh

2. Chạy node:
> ros2 run my_joy_teleop my_teleop_node

Lưu ý: Hãy để terminal này chạy. Đây là bộ não chuyển đổi lệnh điều khiển.

### Terminal 4: Kiểm tra (Debug)
Terminal này dùng để nghe topic /cmd_vel và /odom/unfiltered xác nhận rằng mọi thứ đang hoạt động.

1. Source workspace:
Đảm bảo ở đúng thư mục workspace
> cd ~/workspace/MangoMobileRobot 
> source install/setup.zsh

2. Echo /cmd_vel:
> ros2 topic echo /cmd_vel

3. Mở bảng quan sát trực tiếp odometry
ros2 run rqt_gui rqt_gui

Bây giờ, hãy giữ nút an toàn (deadman button) trên tay cầm và di chuyển cần analog. Nếu mọi thứ thành công, bạn sẽ thấy các giá trị linear.x và angular.z xuất hiện trong terminal này.


# Robot UI – Hướng dẫn chạy giao diện hiển thị Map & Robot Pose (ROS 2)

Tài liệu này hướng dẫn **từng bước** để chạy giao diện UI (PyQt6) hiển thị:
- Map (`/map`)
- Robot pose (AMCL)
- Cập nhật realtime trong ROS 2

---

## 1. Yêu cầu hệ thống

- Ubuntu 22.04
- ROS 2 (Humble / Iron / Jazzy)
- Python ≥ 3.10
- PyQt6
- nav2_map_server
- nav2_amcl
- LiDAR driver (vd: sllidar, rplidar)

---

## Thực hiện tác vụ qua hội thoại

Các yêu cầu mở/nghe bài hát sẽ mở Firefox trực tiếp tới kết quả tìm kiếm
YouTube để chọn video; không bật cửa sổ terminal. Yêu cầu mở Terminal hoặc ứng
dụng desktop đã cài được thực hiện trực tiếp; tên ứng dụng được dò từ các
launcher `.desktop`, không chạy lệnh model tạo. Các yêu cầu CLI khác chạy Codex
ở chế độ nền, dùng model `gpt-6-luna`,
`--ephemeral` và sandbox `workspace-write` chỉ trong thư mục project. Codex
không được dùng `sudo`, cài phần mềm, commit/push, truy cập file ngoài project
hoặc vượt sandbox. Tác vụ cần mạng/quyền ngoài sandbox sẽ dừng và báo giới hạn,
không có terminal tương tác để xin thêm quyền. Các thao tác chat này không hiện
hộp thoại xác nhận.

Gửi trực tiếp đường dẫn tuyệt đối tới thư mục trong project, home hoặc ổ đĩa
được mount dưới `/media/<user>` hay `/mnt` (ví dụ
`/home/khoaiuh/zackon_build_up`) sẽ mở thư mục đó trong Ubuntu File Manager qua
`xdg-open`. Lệnh kiểu "mở thư mục <tên>" cũng được phân giải
trong các vị trí này. Ubuntu không có ổ `C:` mặc định; hãy chỉ đường dẫn mount
thực tế của phân vùng nếu muốn mở nó. Thao tác mở thư mục không chạy Codex.

Yêu cầu mở website của một tổ chức, trường, khoa, bộ môn hoặc dịch vụ bất kỳ sẽ
mở kết quả tìm kiếm cho website chính thức tương ứng trong Firefox. Ứng dụng
dùng một địa chỉ tìm kiếm cố định và không tự đoán/mở domain do model trả về;
người dùng chọn đúng kết quả cần truy cập. Cách này không cần thêm danh sách
website vào mã nguồn cho từng khoa.

Ứng dụng từ chối các yêu cầu nhận diện được là tìm/mở nội dung tình dục 18+,
kiểm tra lại cả cụm tìm kiếm do model tạo, thêm SafeSearch cho Google/YouTube,
và đưa cùng giới hạn vào chính sách hội thoại và tác vụ Codex. Khi người dùng
hỏi tiếp danh sách/tên tác phẩm của một người đã được xác định là nghệ sĩ nội
dung người lớn, ứng dụng từ chối trước khi gọi model hoặc tìm web. Đây là lớp bảo vệ
trong ứng dụng, không phải bộ lọc mạng toàn hệ thống: không thể bảo đảm chặn mọi
website người lớn mới hoặc URL người dùng tự nhập trong trình duyệt. Muốn chặn
toàn bộ trên máy cần bật chế độ lọc nội dung ở cấp DNS/router/hệ điều hành và
trình duyệt.

Codex CLI cần được cài và đăng nhập bằng tài khoản ChatGPT hoặc API key. Firefox
cần có trên PATH để mở kết quả tìm kiếm bài hát; Ubuntu `xdg-open` được dùng
để mở thư mục.

### Theo dõi người với D435i

Chế độ **Theo dõi** mặc định lấy luồng màu bằng RealSense SDK (`camera_url:=realsense`),
không dùng chỉ số camera cố định. Popup hiển thị bản đồ và trạng thái camera/detector;
**Đóng** hoặc Escape dừng theo dõi và quay lại startup đang mở. Chuyển động mặc định
tắt, cần bật rõ ràng khi thử robot. Xem [hướng dẫn D435i, tham số và nghiệm thu](src/human_following/README.md).
