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
    throw std::runtime_error(
      "[LidarIntensityDock] cannot lock node");
  }

  // Declare parameters
  auto declare = [&](const std::string & key, auto val) {
    node->declare_parameter(
      name_ + "." + key, rclcpp::ParameterValue(val));
  };

  declare("scan_topic",          std::string("/scan_front_filter"));
  declare("base_frame",          std::string("base_link"));
  declare("lrf_tilt_alpha_deg",  0.0);
  declare("lrf_forward_offset",  0.30);
  declare("tape_distance",       0.37);
  declare("rubber_width",        0.020);
  declare("reflector_width",     0.010);
  declare("i_peak",              46.0);
  declare("i_valley",            60.0);
  declare("valley_search_range", 15);
  declare("staging_x_offset",   -0.25);
  declare("docking_threshold",   0.05);

  // Read parameters
  auto get_d = [&](const std::string & k) {
    return node->get_parameter(name_ + "." + k).as_double();
  };

  scan_topic_ = node->get_parameter(
    name_ + ".scan_topic").as_string();
  base_frame_ = node->get_parameter(
    name_ + ".base_frame").as_string();

  lrf_tilt_alpha_     = angles::from_degrees(
                          get_d("lrf_tilt_alpha_deg"));
  lrf_forward_offset_ = get_d("lrf_forward_offset");
  tape_distance_      = get_d("tape_distance");
  rubber_width_       = get_d("rubber_width");
  reflector_width_    = get_d("reflector_width");
  i_peak_             = static_cast<float>(get_d("i_peak"));
  i_valley_           = static_cast<float>(get_d("i_valley"));
  valley_search_range_ = static_cast<int>(
    node->get_parameter(
      name_ + ".valley_search_range").as_int());
  staging_x_offset_   = get_d("staging_x_offset");
  docking_threshold_  = get_d("docking_threshold");

  RCLCPP_INFO(node->get_logger(),
    "[%s] Configured!\n"
    "  scan=%s  base=%s\n"
    "  dL=%.3fm  d=%.3fm\n"
    "  i_peak=%.0f  i_valley=%.0f",
    name_.c_str(),
    scan_topic_.c_str(), base_frame_.c_str(),
    lrf_forward_offset_, tape_distance_,
    static_cast<double>(i_peak_),
    static_cast<double>(i_valley_));
}

void LidarIntensityDock::cleanup()
{
  scan_sub_.reset();
}

void LidarIntensityDock::activate()
{
  auto node = node_.lock();
  if (!node) { return; }

  auto cb = std::bind(&LidarIntensityDock::scanCallback, this, std::placeholders::_1);

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS(), cb);

  RCLCPP_INFO(node->get_logger(), "[%s] Active - listening to %s", name_.c_str(), scan_topic_.c_str());
}

void LidarIntensityDock::deactivate()
{
  scan_sub_.reset();
}

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

  staging.pose.position.x +=
    staging_x_offset_ * std::cos(yaw);
  staging.pose.position.y +=
    staging_x_offset_ * std::sin(yaw);

  return staging;
}

bool LidarIntensityDock::getRefinedPose(
  geometry_msgs::msg::PoseStamped & pose,
  std::string /*id*/)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!dock_detected_) { return false; }
  pose = last_detected_pose_;
  return true;
}

bool LidarIntensityDock::isDocked()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!dock_detected_) { return false; }

  double dist = std::hypot(
    last_detected_pose_.pose.position.x,
    last_detected_pose_.pose.position.y);

  return dist < docking_threshold_;
}

bool LidarIntensityDock::isCharging()
{
  // TODO: subscribe battery topic
  return isDocked();
}

bool LidarIntensityDock::disableCharging()
{
  // TODO: send relay command
  return true;
}

bool LidarIntensityDock::hasStoppedCharging()
{
  return !isDocked();
}

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
    // RPLidar A2:
    // beam nhỏ = lệch PHẢI
    // beam lớn = lệch TRÁI
    const Reflector & right =
      (reflectors[0].peak_idx < reflectors[1].peak_idx)
      ? reflectors[0] : reflectors[1];

    const Reflector & left =
      (reflectors[0].peak_idx < reflectors[1].peak_idx)
      ? reflectors[1] : reflectors[0];

    found = computeDockPose(
      left, right,
      tape_distance_,
      lrf_forward_offset_,
      detected);
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  dock_detected_ = found;
  if (found) { last_detected_pose_ = detected; }
}

// ─────────────────────────────────────────────
// detectReflectors() - Fig.3 của paper
// ─────────────────────────────────────────────

std::vector<LidarIntensityDock::Reflector>
LidarIntensityDock::detectReflectors(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Reflector> result;
  const size_t N = scan.ranges.size();

  if (scan.intensities.size() != N) { return result; }

  const double theta = scan.angle_increment;

  // Max detection range từ Eq.(8a)
  const double max_range =
    std::min(rubber_width_, reflector_width_) /
    (2.0 * std::sin(theta / 2.0));

  const int margin = valley_search_range_ + 2;

  for (int i = margin;
       i < static_cast<int>(N) - margin; ++i)
  {
    const float I_i = scan.intensities[i];

    // Step 1: Peak candidate?
    if (I_i < i_peak_) { continue; }

    // Step 2: Local maximum?
    bool is_max = true;
    for (int k = i - 1;
         k >= std::max(0, i - valley_search_range_); --k) {
      if (scan.intensities[k] > I_i) {
        is_max = false; break;
      }
    }
    if (!is_max) { continue; }

    for (int k = i + 1;
         k <= std::min(static_cast<int>(N) - 1,
                       i + valley_search_range_); ++k) {
      if (scan.intensities[k] > I_i) {
        is_max = false; break;
      }
    }
    if (!is_max) { continue; }

    // Step 3: Tìm valley trái
    int   vl_idx = i - 1;
    float vl_val = scan.intensities[vl_idx];
    for (int k = i - 2;
         k >= std::max(0, i - valley_search_range_); --k) {
      if (scan.intensities[k] < vl_val) {
        vl_val = scan.intensities[k];
        vl_idx = k;
      }
    }

    // Step 4: Tìm valley phải
    int   vr_idx = i + 1;
    float vr_val = scan.intensities[vr_idx];
    for (int k = i + 2;
         k <= std::min(static_cast<int>(N) - 1,
                       i + valley_search_range_); ++k) {
      if (scan.intensities[k] < vr_val) {
        vr_val = scan.intensities[k];
        vr_idx = k;
      }
    }

    // Step 5: Valley hợp lệ?
    if (vl_val > i_valley_ || vr_val > i_valley_) {
      continue;
    }

    // Step 6: Range hợp lệ? Eq.(8a)
    const float r_i = scan.ranges[i];
    if (!std::isfinite(r_i) ||
        r_i < scan.range_min ||
        r_i > scan.range_max) { continue; }
    if (static_cast<double>(r_i) > max_range) { continue; }

    // Step 7: Inception angle β Eq.(3a)
    const float r_j = scan.ranges[i - 1];
    if (!std::isfinite(r_j) ||
        r_j < scan.range_min ||
        r_j > scan.range_max) { continue; }

    double beta = computeInceptionAngle(
      static_cast<double>(r_i),
      static_cast<double>(r_j),
      theta);

    // Remark 2: β quá nhỏ = grazing angle
    if (std::abs(beta) < 0.05) { continue; }

    // Step 8: Tính vị trí Cartesian
    // RPLidar: 0° = -X = thẳng trước
    // Convert sang ROS convention
    double rplidar_angle = scan.angle_min +
                           i * scan.angle_increment;
    double ros_angle = M_PI - rplidar_angle;

    // Normalize [-π, π]
    while (ros_angle >  M_PI) ros_angle -= 2 * M_PI;
    while (ros_angle < -M_PI) ros_angle += 2 * M_PI;

    double x = static_cast<double>(r_i) * std::cos(ros_angle);
    double y = static_cast<double>(r_i) * std::sin(ros_angle);

    Reflector ref;
    ref.peak_idx     = i;
    ref.valley_l_idx = vl_idx;
    ref.valley_r_idx = vr_idx;
    ref.I_peak       = I_i;
    ref.I_valley     = std::min(vl_val, vr_val);
    ref.L_peak       = static_cast<double>(r_i);
    ref.beta         = beta;
    ref.x            = x;
    ref.y            = y;

    result.push_back(ref);

    // Skip qua valley phải tránh double-detect
    i = vr_idx;
  }

  return result;
}

// ─────────────────────────────────────────────
// Inception angle - Eq.(3a) của paper
// β = arctan(Lj×sin(θ) / (Lj×cos(θ) - Li))
// ─────────────────────────────────────────────

double LidarIntensityDock::computeInceptionAngle(
  double Li, double Lj, double theta_rad) const
{
  double sin_t = std::sin(theta_rad);
  double cos_t = std::cos(theta_rad);
  double denom = Lj * cos_t - Li;

  if (std::abs(denom) < 1e-6) { return 0.0; }

  return std::atan(Lj * sin_t / denom);
}

// ─────────────────────────────────────────────
// Dock pose - Eq.(9) của paper
// φm = π/2 - β_L - θ_L
// ─────────────────────────────────────────────

bool LidarIntensityDock::computeDockPose(
  const Reflector & left_tape,
  const Reflector & right_tape,
  double /*tape_dist*/,
  double lrf_offset,
  geometry_msgs::msg::PoseStamped & pose_out) const
{
  // θ_L và θ_R từ vị trí Cartesian
  double theta_L = std::atan2(left_tape.y,  left_tape.x);
  double theta_R = std::atan2(right_tape.y, right_tape.x);

  // φm từ Eq.(9c) - trung bình L và R
  double phi_L = M_PI / 2.0 - left_tape.beta  - theta_L;
  double phi_R = M_PI / 2.0 - right_tape.beta - theta_R;
  double phi_m = (phi_L + phi_R) / 2.0;

  // Tâm dock = midpoint 2 tape + offset LiDAR
  double x_dock = (left_tape.x + right_tape.x) / 2.0
                  + lrf_offset;
  double y_dock = (left_tape.y + right_tape.y) / 2.0;

  pose_out.pose.position.x = x_dock;
  pose_out.pose.position.y = y_dock;
  pose_out.pose.position.z = 0.0;

  pose_out.pose.orientation.x = 0.0;
  pose_out.pose.orientation.y = 0.0;
  pose_out.pose.orientation.z = std::sin(phi_m / 2.0);
  pose_out.pose.orientation.w = std::cos(phi_m / 2.0);

  return true;
}

}  // namespace lidar_dock_detector

// ── pluginlib export ──────────────────────────
PLUGINLIB_EXPORT_CLASS(
  lidar_dock_detector::LidarIntensityDock,
  opennav_docking_core::ChargingDock)