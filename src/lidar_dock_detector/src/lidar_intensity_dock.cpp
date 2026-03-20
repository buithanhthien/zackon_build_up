#include "lidar_dock_detector/lidar_intensity_dock.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lidar_dock_detector
{

// ─────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────

void LidarIntensityDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  node_ = parent;
  name_ = name;
  tf_   = tf;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("[LidarIntensityDock] cannot lock node");
  }

  auto declare = [&](const std::string & key, auto val) {
    node->declare_parameter(name_ + "." + key, rclcpp::ParameterValue(val));
  };

  declare("scan_topic",                  std::string("/scan_front_filter"));
  declare("base_frame",                  std::string("base_link"));
  declare("lrf_tilt_alpha_deg",          0.0);
  declare("lrf_forward_offset",          0.30);
  declare("tape_distance",               0.37);
  declare("rubber_width",                0.020);
  declare("reflector_width",             0.010);
  declare("i_peak",                      60.0);
  declare("i_valley",                    46.0);
  declare("valley_search_range",         15);
  declare("max_detect_range",            3.0);
  declare("max_fail_count",              5);
  declare("staging_x_offset",           -0.8);
  declare("staging_yaw_offset",          0.0);
  declare("docking_threshold",           0.05);
  declare("use_external_detection_pose", false);

  auto get_d = [&](const std::string & k) {
    return node->get_parameter(name_ + "." + k).as_double();
  };
  auto get_i = [&](const std::string & k) {
    return static_cast<int>(node->get_parameter(name_ + "." + k).as_int());
  };
  auto get_b = [&](const std::string & k) {
    return node->get_parameter(name_ + "." + k).as_bool();
  };

  scan_topic_  = node->get_parameter(name_ + ".scan_topic").as_string();
  base_frame_  = node->get_parameter(name_ + ".base_frame").as_string();

  lrf_tilt_alpha_            = angles::from_degrees(get_d("lrf_tilt_alpha_deg"));
  lrf_forward_offset_        = get_d("lrf_forward_offset");
  tape_distance_             = get_d("tape_distance");
  rubber_width_              = get_d("rubber_width");
  reflector_width_           = get_d("reflector_width");
  i_peak_                    = static_cast<float>(get_d("i_peak"));
  i_valley_                  = static_cast<float>(get_d("i_valley"));
  valley_search_range_       = get_i("valley_search_range");
  max_detect_range_          = get_d("max_detect_range");
  max_fail_count_            = get_i("max_fail_count");
  staging_x_offset_          = get_d("staging_x_offset");
  staging_yaw_offset_        = get_d("staging_yaw_offset");
  docking_threshold_         = get_d("docking_threshold");
  use_external_detection_pose_ = get_b("use_external_detection_pose");

  RCLCPP_INFO(node->get_logger(),
    "[%s] Configured: scan=%s base=%s lrf_offset=%.3f tape=%.3f "
    "i_peak=%.0f i_valley=%.0f max_range=%.1f max_fail=%d "
    "staging_x=%.3f staging_yaw=%.3f use_ext=%s",
    name_.c_str(), scan_topic_.c_str(), base_frame_.c_str(),
    lrf_forward_offset_, tape_distance_,
    static_cast<double>(i_peak_), static_cast<double>(i_valley_),
    max_detect_range_, max_fail_count_,
    staging_x_offset_, staging_yaw_offset_,
    use_external_detection_pose_ ? "true" : "false");
}

void LidarIntensityDock::cleanup()  { scan_sub_.reset(); }

void LidarIntensityDock::activate()
{
  auto node = node_.lock();
  if (!node) { return; }
  // Reset state
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    dock_detected_ = false;
    miss_count_    = 0;
    last_detected_pose_ = geometry_msgs::msg::PoseStamped{};  // stamp.sec == 0 sentinel
  }
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LidarIntensityDock::scanCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node->get_logger(), "[%s] Active - listening to %s",
    name_.c_str(), scan_topic_.c_str());
}

void LidarIntensityDock::deactivate() { scan_sub_.reset(); }

// ─────────────────────────────────────────────
// ChargingDock interface
// ─────────────────────────────────────────────

geometry_msgs::msg::PoseStamped
LidarIntensityDock::getStagingPose(
  const geometry_msgs::msg::Pose & dock_pose,
  const std::string & frame)
{
  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = frame;
  staging.header.stamp    = rclcpp::Clock().now();
  staging.pose            = dock_pose;

  double yaw = 2.0 * std::atan2(
    dock_pose.orientation.z,
    dock_pose.orientation.w);

  // Apply x offset along dock heading
  staging.pose.position.x += staging_x_offset_ * std::cos(yaw);
  staging.pose.position.y += staging_x_offset_ * std::sin(yaw);

  // Apply yaw offset to staging orientation
  double staging_yaw = yaw + staging_yaw_offset_;
  staging.pose.orientation.x = 0.0;
  staging.pose.orientation.y = 0.0;
  staging.pose.orientation.z = std::sin(staging_yaw / 2.0);
  staging.pose.orientation.w = std::cos(staging_yaw / 2.0);

  return staging;
}

bool LidarIntensityDock::getRefinedPose(
  geometry_msgs::msg::PoseStamped & pose,
  std::string /*id*/)
{
  // If external detection is disabled, return the database dock pose as-is
  // (pose is already set by the docking server from the dock database)
  if (!use_external_detection_pose_) {
    return true;
  }

  // use_external_detection_pose: true → use LiDAR-detected pose
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!dock_detected_) { return false; }

  // Transform detected pose from laser frame → base_frame_
  try {
    geometry_msgs::msg::PoseStamped pose_in_base;
    tf_->transform(last_detected_pose_, pose_in_base, base_frame_,
                   tf2::durationFromSec(0.1));
    pose = pose_in_base;
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "[%s] TF transform failed: %s", name_.c_str(), ex.what());
    }
    return false;
  }

  return true;
}

bool LidarIntensityDock::isDocked()
{
  // When not using external detection (rear/backward docking), the front LiDAR
  // may lose sight of the reflector during the final backing phase.
  // In that case, rely on the debounced dock_detected_ latch: once the robot
  // was close enough and detection is lost, treat it as docked.
  if (!use_external_detection_pose_) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    // If we never detected the dock at all, not docked.
    if (last_detected_pose_.header.stamp.sec == 0) { return false; }
    // If detection is still active, check distance in base_frame_ (no double offset:
    // computeDockPose already embeds lrf_forward_offset_ into pose.position.x).
    if (dock_detected_) {
      try {
        geometry_msgs::msg::PoseStamped pose_base;
        tf_->transform(last_detected_pose_, pose_base, base_frame_,
                       tf2::durationFromSec(0.1));
        double dist = std::hypot(pose_base.pose.position.x, pose_base.pose.position.y);
        return dist < docking_threshold_;
      } catch (const tf2::TransformException &) {}
    }
    // Detection lost during final backing phase → consider docked
    return true;
  }

  // External detection mode: must have a current detection in base_frame_
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!dock_detected_) { return false; }
  try {
    geometry_msgs::msg::PoseStamped pose_base;
    tf_->transform(last_detected_pose_, pose_base, base_frame_,
                   tf2::durationFromSec(0.1));
    double dist = std::hypot(pose_base.pose.position.x, pose_base.pose.position.y);
    return dist < docking_threshold_;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool LidarIntensityDock::isCharging()         { return isDocked(); }
bool LidarIntensityDock::disableCharging()    { return true; }
bool LidarIntensityDock::hasStoppedCharging() { return !isDocked(); }

// ─────────────────────────────────────────────
// Scan callback
// ─────────────────────────────────────────────

void LidarIntensityDock::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto reflectors = detectReflectors(*msg);

  geometry_msgs::msg::PoseStamped detected;
  detected.header = msg->header;

  bool found = false;

  if (reflectors.size() == 2u) {
    const Reflector & right =
      (reflectors[0].peak_idx < reflectors[1].peak_idx)
      ? reflectors[0] : reflectors[1];
    const Reflector & left =
      (reflectors[0].peak_idx < reflectors[1].peak_idx)
      ? reflectors[1] : reflectors[0];

    found = computeDockPose(left, right, tape_distance_, lrf_forward_offset_, detected);
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (found) {
    miss_count_        = 0;
    dock_detected_     = true;
    last_detected_pose_ = detected;
  } else {
    // Debounce: only clear detection after max_fail_count_ consecutive misses
    if (++miss_count_ > max_fail_count_) {
      dock_detected_ = false;
    }
  }
}

// ─────────────────────────────────────────────
// detectReflectors()
// ─────────────────────────────────────────────

std::vector<LidarIntensityDock::Reflector>
LidarIntensityDock::detectReflectors(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Reflector> result;
  const size_t N = scan.ranges.size();
  if (scan.intensities.size() != N) { return result; }

  const double theta = scan.angle_increment;

  // Geometric max range from Eq.(8a), capped by configured max_detect_range_
  const double max_range_geom =
    std::min(rubber_width_, reflector_width_) / (2.0 * std::sin(theta / 2.0));
  const double max_range = std::min(max_range_geom, max_detect_range_);

  const int margin = valley_search_range_ + 2;

  for (int i = margin; i < static_cast<int>(N) - margin; ++i) {
    const float I_i = scan.intensities[i];
    if (I_i < i_peak_) { continue; }

    // Local maximum check
    bool is_max = true;
    for (int k = i - 1; k >= std::max(0, i - valley_search_range_); --k) {
      if (scan.intensities[k] > I_i) { is_max = false; break; }
    }
    if (!is_max) { continue; }
    for (int k = i + 1; k <= std::min(static_cast<int>(N) - 1, i + valley_search_range_); ++k) {
      if (scan.intensities[k] > I_i) { is_max = false; break; }
    }
    if (!is_max) { continue; }

    // Left valley
    int   vl_idx = i - 1;
    float vl_val = scan.intensities[vl_idx];
    for (int k = i - 2; k >= std::max(0, i - valley_search_range_); --k) {
      if (scan.intensities[k] < vl_val) { vl_val = scan.intensities[k]; vl_idx = k; }
    }

    // Right valley
    int   vr_idx = i + 1;
    float vr_val = scan.intensities[vr_idx];
    for (int k = i + 2; k <= std::min(static_cast<int>(N) - 1, i + valley_search_range_); ++k) {
      if (scan.intensities[k] < vr_val) { vr_val = scan.intensities[k]; vr_idx = k; }
    }

    if (vl_val > i_valley_ || vr_val > i_valley_) { continue; }

    const float r_i = scan.ranges[i];
    if (!std::isfinite(r_i) || r_i < scan.range_min || r_i > scan.range_max) { continue; }
    if (static_cast<double>(r_i) > max_range) { continue; }

    const float r_j = scan.ranges[i - 1];
    if (!std::isfinite(r_j) || r_j < scan.range_min || r_j > scan.range_max) { continue; }

    double beta = computeInceptionAngle(
      static_cast<double>(r_i), static_cast<double>(r_j), theta);
    if (std::abs(beta) < 0.05) { continue; }

    double rplidar_angle = scan.angle_min + i * scan.angle_increment;
    double ros_angle = M_PI - rplidar_angle;
    while (ros_angle >  M_PI) ros_angle -= 2 * M_PI;
    while (ros_angle < -M_PI) ros_angle += 2 * M_PI;

    Reflector ref;
    ref.peak_idx     = i;
    ref.valley_l_idx = vl_idx;
    ref.valley_r_idx = vr_idx;
    ref.I_peak       = I_i;
    ref.I_valley     = std::min(vl_val, vr_val);
    ref.L_peak       = static_cast<double>(r_i);
    ref.beta         = beta;
    ref.x            = static_cast<double>(r_i) * std::cos(ros_angle);
    ref.y            = static_cast<double>(r_i) * std::sin(ros_angle);

    result.push_back(ref);
    i = vr_idx;
  }

  return result;
}

// ─────────────────────────────────────────────
// Inception angle - Eq.(3a)
// ─────────────────────────────────────────────

double LidarIntensityDock::computeInceptionAngle(
  double Li, double Lj, double theta_rad) const
{
  double denom = Lj * std::cos(theta_rad) - Li;
  if (std::abs(denom) < 1e-6) { return 0.0; }
  return std::atan(Lj * std::sin(theta_rad) / denom);
}

// ─────────────────────────────────────────────
// Dock pose - Eq.(9)
// ─────────────────────────────────────────────

bool LidarIntensityDock::computeDockPose(
  const Reflector & left_tape,
  const Reflector & right_tape,
  double tape_dist,
  double lrf_offset,
  geometry_msgs::msg::PoseStamped & pose_out) const
{
  // Validate reflector pair spacing against configured tape_distance
  double measured_dist = std::hypot(
    left_tape.x - right_tape.x, left_tape.y - right_tape.y);
  if (std::abs(measured_dist - tape_dist) > tape_dist * 0.4) {
    return false;  // geometry inconsistent — likely false positive pair
  }

  double theta_L = std::atan2(left_tape.y,  left_tape.x);
  double theta_R = std::atan2(right_tape.y, right_tape.x);

  double phi_L = M_PI / 2.0 - left_tape.beta  - theta_L;
  double phi_R = M_PI / 2.0 - right_tape.beta - theta_R;
  double phi_m = (phi_L + phi_R) / 2.0;

  // Midpoint of two tapes + LiDAR forward offset (single compensation point)
  pose_out.pose.position.x = (left_tape.x + right_tape.x) / 2.0 + lrf_offset;
  pose_out.pose.position.y = (left_tape.y + right_tape.y) / 2.0;
  pose_out.pose.position.z = 0.0;

  pose_out.pose.orientation.x = 0.0;
  pose_out.pose.orientation.y = 0.0;
  pose_out.pose.orientation.z = std::sin(phi_m / 2.0);
  pose_out.pose.orientation.w = std::cos(phi_m / 2.0);

  return true;
}

}  // namespace lidar_dock_detector

PLUGINLIB_EXPORT_CLASS(
  lidar_dock_detector::LidarIntensityDock,
  opennav_docking_core::ChargingDock)
