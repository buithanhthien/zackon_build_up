#ifndef LIDAR_DOCK_DETECTOR__BLIND_DOCKING_HPP_
#define LIDAR_DOCK_DETECTOR__BLIND_DOCKING_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "opennav_docking_core/charging_dock.hpp"

namespace lidar_dock_detector
{

/**
 * BlindDocking — sensor-free dead-reckoning dock plugin.
 *
 * The robot navigates to a staging pose facing AWAY from the dock
 * (lidar faces away), then reverses straight into the dock using
 * only odometry. No sensor is used during the final approach.
 */
class BlindDocking : public opennav_docking_core::ChargingDock
{
public:
  BlindDocking() = default;
  ~BlindDocking() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

  void cleanup()    override {}
  void activate()   override {}
  void deactivate() override {}

  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & dock_pose,
    const std::string & frame) override;

  bool getRefinedPose(
    geometry_msgs::msg::PoseStamped & pose,
    std::string /*id*/) override;

  bool isDocked()           override;
  bool isCharging()         override { return isDocked(); }
  bool disableCharging()    override { return true; }
  bool hasStoppedCharging() override { return !isDocked(); }

private:
  std::string name_;
  std::string base_frame_;
  double staging_x_offset_;   // negative = staging pose is behind the dock
  double staging_yaw_offset_;
  double docking_threshold_;

  geometry_msgs::msg::PoseStamped dock_pose_;  // cached dock pose in fixed frame

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace lidar_dock_detector

#endif  // LIDAR_DOCK_DETECTOR__BLIND_DOCKING_HPP_
