#ifndef LIDAR_DOCK_DETECTOR__DOCKING_NODE_HPP_
#define LIDAR_DOCK_DETECTOR__DOCKING_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace lidar_dock_detector
{

enum class State {
  IDLE,
  NAVIGATE_TO_STAGING,
  DETECT_DOCK,
  ROTATE_180,
  MOVE_BACKWARD,
  DOCKED,
  FAILED
};

struct Reflector {
  int    peak_idx;
  double x, y;
};

class DockingNode : public rclcpp::Node
{
public:
  explicit DockingNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

private:
  // callbacks
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void triggerDocking(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void controlLoop();

  // state handlers
  void handleNavigate();
  void handleDetect();
  void handleRotate();
  void handleBackup();

  // detection
  std::vector<Reflector> detectReflectors(const sensor_msgs::msg::LaserScan & scan) const;
  void publishZeroVel();

  // parameters
  std::string scan_topic_;
  std::string base_frame_;
  double i_peak_;
  double i_valley_;
  int    valley_search_range_;
  double max_detect_range_;
  int    max_fail_count_;
  double lrf_forward_offset_;
  double tape_distance_;
  double rotate_speed_;
  double backup_speed_;
  double backup_distance_;
  double staging_pose_x_;
  double staging_pose_y_;
  double staging_pose_yaw_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  using NavToPose = nav2_msgs::action::NavigateToPose;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;

  // state
  State state_{State::IDLE};
  std::mutex scan_mutex_;
  std::mutex odom_mutex_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Odometry::SharedPtr    latest_odom_;

  // detect state
  int fail_count_{0};
  int retry_count_{0};

  // rotate state
  double rotate_start_yaw_{0.0};
  bool   rotate_started_{false};

  // backup state
  double backup_start_x_{0.0};
  double backup_start_y_{0.0};
  bool   backup_started_{false};

  // nav state
  bool nav_goal_sent_{false};
  bool nav_done_{false};
  bool nav_success_{false};
};

}  // namespace lidar_dock_detector

#endif  // LIDAR_DOCK_DETECTOR__DOCKING_NODE_HPP_
