import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self.subscription = self.create_subscription(
            Odometry,
            'odomfromSTM32',
            self.odom_callback,
            qos
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odom TF Broadcaster started (odom → base_link) + republish /odom")

    def odom_callback(self, msg: Odometry):
        # Use STM32 stamp if valid; else use a single 'now' captured once
        
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        else:
            stamp = msg.header.stamp

        # Republish /odom (Nav2 expects these frames to match TF)
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

        # Broadcast TF with the SAME stamp
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
