#ifndef LIDAR_DOCK_DETECTOR__CUSTOM_RANGE_DOCK_HPP_
#define LIDAR_DOCK_DETECTOR__CUSTOM_RANGE_DOCK_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace lidar_dock_detector
{

class CustomRangeDock : public rclcpp::Node
{
public:
  explicit CustomRangeDock(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
  void startDocking();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle);
  void feedbackCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
  void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
  
  geometry_msgs::msg::PoseStamped computeStagingPose();
  void publishMarkers();
  void checkAlignmentAndProceed();
  bool isAlignedWithDock();
  void executeBackwardDocking();
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::TimerBase::SharedPtr backward_timer_;
  rclcpp::TimerBase::SharedPtr staging_timer_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  
  geometry_msgs::msg::Pose home_dock_pose_;
  double staging_x_offset_;
  double target_range_;
  double backward_velocity_;
  double range_tolerance_;
  double alignment_tolerance_;
  
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  bool at_staging_pose_;
  int staging_attempts_;
  int max_staging_attempts_;
};

}  // namespace lidar_dock_detector

#endif  // LIDAR_DOCK_DETECTOR__CUSTOM_RANGE_DOCK_HPP_
