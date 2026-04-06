#include "lidar_dock_detector/blind_docking.hpp"

#include <cmath>
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lidar_dock_detector
{

void BlindDocking::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  node_ = parent;
  name_ = name;
  tf_   = tf;

  auto node = node_.lock();
  if (!node) { throw std::runtime_error("[BlindDocking] cannot lock node"); }

  auto declare = [&](const std::string & key, auto val) {
    node->declare_parameter(name_ + "." + key, rclcpp::ParameterValue(val));
  };

  declare("base_frame",        std::string("base_link"));
  declare("staging_x_offset",  -0.80);
  declare("staging_yaw_offset", 0.0);
  declare("docking_threshold",  0.5);
  // Declared for parameter namespace compatibility
  declare("rotate_to_dock",    false);
  declare("dock_direction",    std::string("backward"));

  base_frame_        = node->get_parameter(name_ + ".base_frame").as_string();
  staging_x_offset_  = node->get_parameter(name_ + ".staging_x_offset").as_double();
  staging_yaw_offset_= node->get_parameter(name_ + ".staging_yaw_offset").as_double();
  docking_threshold_ = node->get_parameter(name_ + ".docking_threshold").as_double();

  RCLCPP_INFO(node->get_logger(),
    "[%s] BlindDocking configured: staging_x=%.3f threshold=%.3f",
    name_.c_str(), staging_x_offset_, docking_threshold_);
}

geometry_msgs::msg::PoseStamped BlindDocking::getStagingPose(
  const geometry_msgs::msg::Pose & dock_pose,
  const std::string & frame)
{
  auto node = node_.lock();

  double yaw = 2.0 * std::atan2(dock_pose.orientation.z, dock_pose.orientation.w);

  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = frame;
  staging.header.stamp    = node ? node->now() : rclcpp::Clock().now();
  staging.pose            = dock_pose;

  // Staging position: behind the dock along dock heading
  staging.pose.position.x += staging_x_offset_ * std::cos(yaw);
  staging.pose.position.y += staging_x_offset_ * std::sin(yaw);

  // Staging orientation: face AWAY from dock (yaw + π) so robot reverses in
  double staging_yaw = yaw + M_PI + staging_yaw_offset_;
  staging.pose.orientation.x = 0.0;
  staging.pose.orientation.y = 0.0;
  staging.pose.orientation.z = std::sin(staging_yaw / 2.0);
  staging.pose.orientation.w = std::cos(staging_yaw / 2.0);

  return staging;
}

bool BlindDocking::getRefinedPose(
  geometry_msgs::msg::PoseStamped & pose,
  std::string /*id*/)
{
  // Dead reckoning: accept the dock pose from the database as-is
  dock_pose_ = pose;
  return true;
}

bool BlindDocking::isDocked()
{
  if (dock_pose_.header.frame_id.empty()) { return false; }

  try {
    geometry_msgs::msg::TransformStamped robot_tf =
      tf_->lookupTransform(dock_pose_.header.frame_id, base_frame_, tf2::TimePointZero);

    double dx = dock_pose_.pose.position.x - robot_tf.transform.translation.x;
    double dy = dock_pose_.pose.position.y - robot_tf.transform.translation.y;
    return std::hypot(dx, dy) < docking_threshold_;

  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(), "[%s] isDocked() TF failed: %s", name_.c_str(), ex.what());
    }
    return false;
  }
}

}  // namespace lidar_dock_detector

PLUGINLIB_EXPORT_CLASS(lidar_dock_detector::BlindDocking, opennav_docking_core::ChargingDock)
