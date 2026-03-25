#ifndef LIDAR_DOCK_DETECTOR__GUESSING_DOCK_HPP_
#define LIDAR_DOCK_DETECTOR__GUESSING_DOCK_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "opennav_docking_core/charging_dock.hpp"
#include "std_msgs/msg/float32.hpp"

namespace lidar_dock_detector
{

/// @brief Sensor-free docking plugin.
///
/// Strategy:
///   1. getStagingPose() places the robot behind the dock, rotated by
///      (staging_yaw_offset + 80°) so the docking server's rotate-to-heading
///      step spins the robot ~80° in place before the backward approach.
///   2. getRefinedPose() accepts the map-frame dock pose as-is (no sensor).
///   3. isDocked() checks the 2-D Euclidean distance from base_link to the
///      dock pose via TF; returns true when dist < docking_threshold.
class GuessingDock : public opennav_docking_core::ChargingDock
{
public:
  GuessingDock()  = default;
  ~GuessingDock() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & dock_pose,
    const std::string & frame) override;

  bool getRefinedPose(
    geometry_msgs::msg::PoseStamped & pose,
    std::string /*id*/) override;

  bool isDocked()           override;
  bool isCharging()         override;
  bool disableCharging()    override;
  bool hasStoppedCharging() override;

private:
  // ── Parameters ──────────────────────────────────────────────────────────────
  std::string name_;             // Plugin instance name assigned by the docking server

  std::string base_frame_;       // Robot base TF frame (e.g. "base_link")
  double staging_x_offset_;      // Longitudinal offset (m) from dock pose to staging pose (negative = behind dock)
  double staging_yaw_offset_;    // Extra yaw offset (rad) added on top of the built-in 180° rotation
  double docking_threshold_;     // 2-D Euclidean distance (m) at which isDocked() returns true

  // 180° yaw added to staging orientation so the robot faces directly backward
  // (away from dock) at the staging pose. The docking server then simply reverses it in.
  static constexpr double kGuessingYawRad = M_PI;  // 180°

  // ── ROS handles ─────────────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dock_distance_pub_;

  // ── State ───────────────────────────────────────────────────────────────────
  geometry_msgs::msg::PoseStamped dock_pose_odom_;  // dock pose in odom frame, set by getRefinedPose
  bool dock_pose_received_{false};                   // true once getRefinedPose has been called
  std::mutex pose_mutex_;
};

}  // namespace lidar_dock_detector

#endif  // LIDAR_DOCK_DETECTOR__GUESSING_DOCK_HPP_
