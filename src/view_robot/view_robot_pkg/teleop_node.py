import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

# === TÙY CHỈNH CÁC CHỈ SỐ NÀY ===
# Tìm các chỉ số này bằng cách chạy `ros2 topic echo /joy`
AXIS_LINEAR = 1       # Chỉ số của trục cho tốc độ tiến/lùi (thường là 1 - cần analog trái lên/xuống)
AXIS_ANGULAR = 0      # Chỉ số của trục cho tốc độ xoay (thường là 0 - cần analog trái trái/phải)
DEADMAN_BUTTON = 4    # Chỉ số của nút "kích hoạt" (thường là 4 - nút L1 hoặc LB)
# =================================

class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('my_joy_teleop_node')
        
        # Tạo một subscriber lắng nghe chủ đề /joy
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Chỉ gửi vào arbiter; không ghi trực tiếp lên đầu ra robot.
        self.publisher = self.create_publisher(Twist, '/cmd_vel_sources/teleop', 1)
        self.release_pub = self.create_publisher(Empty, '/velocity_arbiter/release/teleop', 1)
        self._held = False

        # Khai báo các tham số cho tốc độ tối đa (tùy chọn nhưng nên làm)
        self.declare_parameter('scale_linear', 0.3)  # Tốc độ tiến/lùi tối đa (m/s)
        self.declare_parameter('scale_angular', 0.3141592) # Tốc độ xoay tối đa (rad/s)
        
        self.declare_parameter('deadzone', 0.1)  # Vùng chết 10%

        self.get_logger().info("Node teleop tùy chỉnh đã khởi động!")
        self.get_logger().info(f"Giữ nút [Button {DEADMAN_BUTTON}] để di chuyển.")


    def joy_callback(self, msg):
        """
        Hàm này được gọi mỗi khi có tin nhắn mới trên chủ đề /joy
        """
        # Đọc giá trị của các tham số
        scale_linear = self.get_parameter('scale_linear').value
        scale_angular = self.get_parameter('scale_angular').value

        # Tạo một tin nhắn Twist mới (mặc định là 0)
        twist_msg = Twist()

        # Kiểm tra xem nút "kích hoạt" có đang được giữ không
        held = (len(msg.buttons) > DEADMAN_BUTTON and
                len(msg.axes) > max(AXIS_LINEAR, AXIS_ANGULAR) and
                msg.buttons[DEADMAN_BUTTON] == 1)
        if not held:
            if self._held:
                self.release_pub.publish(Empty())
            self._held = False
            return
        self._held = True
        if held:
            # Ánh xạ giá trị từ trục sang tốc độ
            # msg.axes[] có giá trị từ -1.0 đến 1.0
            twist_msg.linear.x = msg.axes[AXIS_LINEAR] * scale_linear
            twist_msg.angular.z = msg.axes[AXIS_ANGULAR] * scale_angular
        
        # Khi nhả deadman, trả quyền ở nhánh phía trên; không spam zero lúc rảnh.

        # Xuất bản tin nhắn Twist
        self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    
    joy_teleop_node = JoyTeleopNode()
    
    try:
        rclpy.spin(joy_teleop_node)
    except KeyboardInterrupt:
        pass # Cho phép Ctrl+C để tắt node

    joy_teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
