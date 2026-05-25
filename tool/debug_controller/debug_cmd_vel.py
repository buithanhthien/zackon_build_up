#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime

class CmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_logger')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.log_file = open('/home/khoaiuh/zackon_build_up/tool/debug_controller/cmd_vel_log_4.txt', 'a')
        self.get_logger().info('Logging cmd_vel to cmd_vel_log_4.txt')
        
    def callback(self, msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        line = f"{timestamp}, linear.x: {msg.linear.x:.3f}, angular.z: {msg.angular.z:.3f}\n"
        self.log_file.write(line)
        self.log_file.flush()

    def __del__(self):
        self.log_file.close()

def main():
    rclpy.init()
    node = CmdVelLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
