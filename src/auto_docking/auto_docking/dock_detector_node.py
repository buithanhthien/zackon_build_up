#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from sklearn.cluster import DBSCAN

class DockDetectorNode(Node):
    def __init__(self):
        super().__init__('dock_detector_node')
        
        self.declare_parameter('intensity_threshold', 50.0)
        self.declare_parameter('cluster_distance_threshold', 0.05)
        self.declare_parameter('expected_strip_spacing', 0.32)
        self.declare_parameter('spacing_tolerance', 0.05)
        
        self.intensity_threshold = self.get_parameter('intensity_threshold').value
        self.cluster_distance = self.get_parameter('cluster_distance_threshold').value
        self.expected_spacing = self.get_parameter('expected_strip_spacing').value
        self.spacing_tolerance = self.get_parameter('spacing_tolerance').value
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.dock_pose_pub = self.create_publisher(PoseStamped, '/dock_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dock_markers', 10)
        
        self.get_logger().info('Dock Detector Node started')
    
    def scan_callback(self, msg):
        reflective_points = []
        
        for i, intensity in enumerate(msg.intensities):
            if intensity > self.intensity_threshold:
                angle = msg.angle_min + i * msg.angle_increment
                range_val = msg.ranges[i]
                
                if np.isfinite(range_val) and range_val > 0:
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    reflective_points.append([x, y])
        
        if len(reflective_points) < 6:
            return
        
        points = np.array(reflective_points)
        clustering = DBSCAN(eps=self.cluster_distance, min_samples=3).fit(points)
        labels = clustering.labels_
        
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)
        
        if len(unique_labels) != 2:
            return
        
        clusters = []
        for label in unique_labels:
            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            clusters.append(centroid)
        
        P1, P2 = clusters[0], clusters[1]
        distance = np.linalg.norm(P2 - P1)
        
        if abs(distance - self.expected_spacing) > self.spacing_tolerance:
            return
        
        dock_center = (P1 + P2) / 2
        v = P2 - P1
        n = np.array([-v[1], v[0]])
        n = n / np.linalg.norm(n)
        
        dock_pose = PoseStamped()
        dock_pose.header.stamp = self.get_clock().now().to_msg()
        dock_pose.header.frame_id = msg.header.frame_id
        dock_pose.pose.position.x = dock_center[0]
        dock_pose.pose.position.y = dock_center[1]
        dock_pose.pose.position.z = 0.0
        
        yaw = np.arctan2(n[1], n[0])
        dock_pose.pose.orientation.z = np.sin(yaw / 2)
        dock_pose.pose.orientation.w = np.cos(yaw / 2)
        
        self.dock_pose_pub.publish(dock_pose)
        self.publish_markers(P1, P2, dock_center, msg.header.frame_id)
    
    def publish_markers(self, P1, P2, center, frame_id):
        marker_array = MarkerArray()
        
        for i, point in enumerate([P1, P2]):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        
        center_marker = Marker()
        center_marker.header.frame_id = frame_id
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.id = 2
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = center[0]
        center_marker.pose.position.y = center[1]
        center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.15
        center_marker.color.g = 1.0
        center_marker.color.a = 1.0
        marker_array.markers.append(center_marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DockDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
