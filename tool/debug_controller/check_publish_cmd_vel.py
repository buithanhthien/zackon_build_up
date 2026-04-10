import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelChecker(Node):
    def __init__(self):
        super().__init__('cmd_vel_checker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1/20, self.publish_cmd_vel)

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main():
    rclpy.init()
    node = CmdVelChecker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
