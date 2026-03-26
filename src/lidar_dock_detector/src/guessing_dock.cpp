#include "lidar_dock_detector/guessing_dock.hpp"

#include <cmath>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace lidar_dock_detector {

// ─────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────

void GuessingDock::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &name, std::shared_ptr<tf2_ros::Buffer> tf) {
  node_ = parent;
  name_ = name;
  tf_ = tf;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("[GuessingDock] cannot lock node");
  }

  // Declare parameters under the plugin's namespace (name_.<key>)
  auto declare = [&](const std::string &key, auto val) {
    node->declare_parameter(name_ + "." + key, rclcpp::ParameterValue(val));
  };

  // Only the parameters actually used by this plugin
  declare("base_frame", std::string("base_link"));
  declare("staging_x_offset", -0.80); // m, negative = behind dock
  declare("staging_yaw_offset", 0.0); // rad, on top of the 80° rotation
  declare("docking_threshold", 0.45); // m

  // Also declare (but ignore) unused params so YAML loading doesn't error
  // if the same YAML block is reused between plugins
  declare("scan_topic", std::string("/scan_front_filter"));
  declare("lrf_tilt_alpha_deg", 0.0);
  declare("lrf_forward_offset", 0.30);
  declare("tape_distance", 0.375);
  declare("rubber_width", 0.32);
  declare("reflector_width", 0.048);
  declare("i_peak", 43.0);
  declare("i_valley", 29.0);
  declare("valley_search_range", 19);
  declare("max_detect_range", 3.0);
  declare("max_fail_count", 5);
  declare("use_external_detection_pose", false);
  declare("dock_direction", std::string("backward"));
  declare("rotate_to_dock", false);

  auto get_d = [&](const std::string &k) {
    return node->get_parameter(name_ + "." + k).as_double();
  };

  base_frame_ = node->get_parameter(name_ + ".base_frame").as_string();
  staging_x_offset_ = get_d("staging_x_offset");
  staging_yaw_offset_ = get_d("staging_yaw_offset");
  docking_threshold_ = get_d("docking_threshold");

  RCLCPP_INFO(node->get_logger(),
              "[%s] GuessingDock configured: base=%s staging_x=%.3f "
              "staging_yaw=%.3f guessing_yaw=180° threshold=%.3f",
              name_.c_str(), base_frame_.c_str(), staging_x_offset_,
              staging_yaw_offset_, docking_threshold_);
}

void GuessingDock::cleanup() {}
void GuessingDock::deactivate() { dock_distance_pub_.reset(); }

void GuessingDock::activate() {
  auto node = node_.lock();
  if (!node) {
    return;
  }

  // Reset state
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    dock_pose_received_ = false;
    dock_pose_odom_ = geometry_msgs::msg::PoseStamped{};
  }

  dock_distance_pub_ = node->create_publisher<std_msgs::msg::Float32>(
      "/guessing_dock_distance", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(node->get_logger(), "[%s] GuessingDock active", name_.c_str());
}

// ─────────────────────────────────────────────
// ChargingDock interface
// ─────────────────────────────────────────────

// ── getStagingPose()
// ────────────────────────────────────────────────────────── Returns the
// staging pose: the position behind the dock plus a combined yaw of (dock_yaw +
// staging_yaw_offset + kGuessingYawRad).
//
// The extra 80° (kGuessingYawRad) means the robot arrives at the staging
// position facing ~80° away from the dock heading. The docking server's
// rotate-to-heading step then spins it in place before the backward approach.
geometry_msgs::msg::PoseStamped
GuessingDock::getStagingPose(const geometry_msgs::msg::Pose &dock_pose,
                             const std::string &frame) {
  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = frame;
  auto node = node_.lock();
  staging.header.stamp = node ? node->now() : rclcpp::Clock().now();
  staging.pose = dock_pose;

  // Extract dock yaw from quaternion (2-D, z-w only)
  double dock_yaw =
      2.0 * std::atan2(dock_pose.orientation.z, dock_pose.orientation.w);

  // Offset staging position along dock heading
  staging.pose.position.x += staging_x_offset_ * std::cos(dock_yaw);
  staging.pose.position.y += staging_x_offset_ * std::sin(dock_yaw);

  // Combine staging yaw: dock heading + user offset + built-in 80° rotation
  double staging_yaw = dock_yaw + staging_yaw_offset_ + kGuessingYawRad;
  staging.pose.orientation.x = 0.0;
  staging.pose.orientation.y = 0.0;
  staging.pose.orientation.z = std::sin(staging_yaw / 2.0);
  staging.pose.orientation.w = std::cos(staging_yaw / 2.0);

  return staging;
}

// ── getRefinedPose()
// ────────────────────────────────────────────────────────── No sensor — accept
// the dock pose supplied by the server as-is. Cache it in dock_pose_odom_ so
// isDocked() can use it.
bool GuessingDock::getRefinedPose(geometry_msgs::msg::PoseStamped &pose,
                                  std::string /*id*/) {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  dock_pose_odom_ = pose; // latch the pose handed to us
  dock_pose_received_ = true;
  return true; // always accept
}

// ── isDocked()
// ──────────────────────────────────────────────────────────────── Transforms
// the latched dock pose into base_link and checks whether the 2-D Euclidean
// distance is within docking_threshold_.
bool GuessingDock::isDocked() {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!dock_pose_received_) {
    return false;
  }

  try {
    geometry_msgs::msg::PoseStamped pose_base;
    tf_->transform(dock_pose_odom_, pose_base, base_frame_,
                   tf2::durationFromSec(0.1));

    double dist =
        std::hypot(pose_base.pose.position.x, pose_base.pose.position.y);

    // Publish distance for debugging
    if (dock_distance_pub_) {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(dist);
      dock_distance_pub_->publish(msg);
    }

    return dist < docking_threshold_;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool GuessingDock::isCharging() { return isDocked(); }
bool GuessingDock::disableCharging() { return true; }
bool GuessingDock::hasStoppedCharging() { return !isDocked(); }

} // namespace lidar_dock_detector

PLUGINLIB_EXPORT_CLASS(lidar_dock_detector::GuessingDock,
                       opennav_docking_core::ChargingDock)
