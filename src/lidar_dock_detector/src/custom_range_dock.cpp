#include "lidar_dock_detector/custom_range_dock.hpp"
#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lidar_dock_detector
{

CustomRangeDock::CustomRangeDock(const rclcpp::NodeOptions & options)
: Node("custom_range_dock", options), at_staging_pose_(false), staging_attempts_(0), max_staging_attempts_(4)
{
  // Parameters
  this->declare_parameter("home_dock.x", -0.5155331150);
  this->declare_parameter("home_dock.y", 1.5446702719);
  this->declare_parameter("home_dock.yaw", -1.6477);
  this->declare_parameter("staging_x_offset", 0.8);
  this->declare_parameter("target_range", 0.245);
  this->declare_parameter("backward_velocity", -0.3);
  this->declare_parameter("range_tolerance", 0.01);
  this->declare_parameter("alignment_tolerance", 0.1);  // ~5.7 degrees
  
  home_dock_pose_.position.x = this->get_parameter("home_dock.x").as_double();
  home_dock_pose_.position.y = this->get_parameter("home_dock.y").as_double();
  double yaw = this->get_parameter("home_dock.yaw").as_double();
  home_dock_pose_.position.z = 0.0;
  home_dock_pose_.orientation.x = 0.0;
  home_dock_pose_.orientation.y = 0.0;
  home_dock_pose_.orientation.z = std::sin(yaw / 2.0);
  home_dock_pose_.orientation.w = std::cos(yaw / 2.0);
  
  staging_x_offset_ = this->get_parameter("staging_x_offset").as_double();
  target_range_ = this->get_parameter("target_range").as_double();
  backward_velocity_ = this->get_parameter("backward_velocity").as_double();
  range_tolerance_ = this->get_parameter("range_tolerance").as_double();
  alignment_tolerance_ = this->get_parameter("alignment_tolerance").as_double();
  
  // Publishers & Subscribers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/docking_markers", 10);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_rear_lidar_filter", 10,
    std::bind(&CustomRangeDock::scanCallback, this, std::placeholders::_1));
  
  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
  
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  RCLCPP_INFO(this->get_logger(), "Custom Range Dock node initialized");
}

void CustomRangeDock::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
}

geometry_msgs::msg::PoseStamped CustomRangeDock::computeStagingPose()
{
  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = "map";
  staging.header.stamp = this->now();
  
  double yaw = 2.0 * std::atan2(home_dock_pose_.orientation.z, home_dock_pose_.orientation.w);
  
  staging.pose.position.x = home_dock_pose_.position.x + staging_x_offset_ * std::cos(yaw);
  staging.pose.position.y = home_dock_pose_.position.y + staging_x_offset_ * std::sin(yaw);
  staging.pose.position.z = 0.0;
  staging.pose.orientation = home_dock_pose_.orientation;
  
  return staging;
}

void CustomRangeDock::publishMarkers()
{
  // Home dock marker
  visualization_msgs::msg::Marker dock_marker;
  dock_marker.header.frame_id = "map";
  dock_marker.header.stamp = this->now();
  dock_marker.ns = "docking";
  dock_marker.id = 0;
  dock_marker.type = visualization_msgs::msg::Marker::ARROW;
  dock_marker.action = visualization_msgs::msg::Marker::ADD;
  dock_marker.pose = home_dock_pose_;
  dock_marker.scale.x = 0.5;
  dock_marker.scale.y = 0.1;
  dock_marker.scale.z = 0.1;
  dock_marker.color.r = 1.0;
  dock_marker.color.g = 0.0;
  dock_marker.color.b = 0.0;
  dock_marker.color.a = 1.0;
  marker_pub_->publish(dock_marker);
  
  // Staging pose marker
  auto staging = computeStagingPose();
  visualization_msgs::msg::Marker staging_marker;
  staging_marker.header = staging.header;
  staging_marker.ns = "docking";
  staging_marker.id = 1;
  staging_marker.type = visualization_msgs::msg::Marker::ARROW;
  staging_marker.action = visualization_msgs::msg::Marker::ADD;
  staging_marker.pose = staging.pose;
  staging_marker.scale.x = 0.5;
  staging_marker.scale.y = 0.1;
  staging_marker.scale.z = 0.1;
  staging_marker.color.r = 0.0;
  staging_marker.color.g = 1.0;
  staging_marker.color.b = 0.0;
  staging_marker.color.a = 1.0;
  marker_pub_->publish(staging_marker);
}

void CustomRangeDock::startDocking()
{
  RCLCPP_INFO(this->get_logger(), "Starting docking sequence");
  publishMarkers();
  
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
    return;
  }
  
  auto staging_pose = computeStagingPose();
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = staging_pose;
  
  RCLCPP_INFO(this->get_logger(), "Sending goal to staging pose: [%.3f, %.3f]",
    staging_pose.pose.position.x, staging_pose.pose.position.y);
  
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&CustomRangeDock::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&CustomRangeDock::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&CustomRangeDock::resultCallback, this, std::placeholders::_1);
  
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void CustomRangeDock::goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted, navigating to staging pose");
  }
}

void CustomRangeDock::feedbackCallback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Distance remaining: %.2f", feedback->distance_remaining);
}

void CustomRangeDock::resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      staging_attempts_++;
      RCLCPP_INFO(this->get_logger(), "Reached staging pose (attempt %d/%d)", 
        staging_attempts_, max_staging_attempts_);
      
      if (staging_attempts_ < max_staging_attempts_) {
        // Wait 1 second before sending next staging goal
        staging_timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          [this]() {
            RCLCPP_INFO(this->get_logger(), "Sending staging goal again for calibration...");
            auto staging_pose = computeStagingPose();
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose = staging_pose;
            
            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&CustomRangeDock::goalResponseCallback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&CustomRangeDock::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback = std::bind(&CustomRangeDock::resultCallback, this, std::placeholders::_1);
            
            nav_client_->async_send_goal(goal_msg, send_goal_options);
            staging_timer_->cancel();
          });
      } else {
        // After 4th success, stop and wait 3 seconds
        RCLCPP_INFO(this->get_logger(), "Calibration complete. Stopping robot for 3s...");
        at_staging_pose_ = true;
        
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        
        wait_timer_ = this->create_wall_timer(
          std::chrono::seconds(3),
          [this]() { 
            RCLCPP_INFO(this->get_logger(), "Starting backward docking");
            executeBackwardDocking();
            wait_timer_->cancel();
          });
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Navigation canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

// COMMENTED OUT - Alignment check for future use
/*
bool CustomRangeDock::isAlignedWithDock()
{
  try {
    auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    
    double robot_yaw = 2.0 * std::atan2(transform.transform.rotation.z, transform.transform.rotation.w);
    double dock_yaw = 2.0 * std::atan2(home_dock_pose_.orientation.z, home_dock_pose_.orientation.w);
    
    double yaw_diff = std::abs(robot_yaw - dock_yaw);
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    
    RCLCPP_INFO(this->get_logger(), "Alignment: yaw_diff = %.3f rad (%.1f deg), tolerance = %.3f rad",
      std::abs(yaw_diff), std::abs(yaw_diff) * 180.0 / M_PI, alignment_tolerance_);
    
    return std::abs(yaw_diff) < alignment_tolerance_;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
    return false;
  }
}

void CustomRangeDock::checkAlignmentAndProceed()
{
  if (isAlignedWithDock()) {
    RCLCPP_INFO(this->get_logger(), "Robot aligned, starting backward docking");
    executeBackwardDocking();
  } else {
    RCLCPP_WARN(this->get_logger(), "Robot NOT aligned - calibration needed!");
    RCLCPP_INFO(this->get_logger(), "TODO: Implement tape calibration. Proceeding anyway...");
    executeBackwardDocking();
  }
}
*/

void CustomRangeDock::executeBackwardDocking()
{
  backward_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      if (!latest_scan_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No scan data");
        return;
      }
      
      double min_range = *std::min_element(latest_scan_->ranges.begin(), latest_scan_->ranges.end());
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Current range: %.3f, Target: %.3f", min_range, target_range_);
      
      if (min_range <= target_range_ + range_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Docking complete! Stopping robot.");
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        backward_timer_->cancel();
        return;
      }
      
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = backward_velocity_;
      cmd_vel_pub_->publish(cmd);
    });
}

}  // namespace lidar_dock_detector
