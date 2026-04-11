#include "lidar_dock_detector/lidar_intensity_dock.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

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

  declare("scan_topic",                  std::string("/scan_rear_lidar_filter"));
  declare("base_frame",                  std::string("base_link"));
  declare("lrf_tilt_alpha_deg",          0.0);
  declare("lrf_forward_offset",         -0.35);
  declare("tape_distance",               0.375);
  declare("rubber_width",                0.32);
  declare("reflector_width",             0.048);
  
  // NEW: Cluster-based detection parameters
  declare("intensity_cluster_threshold", 45.0);
  declare("min_cluster_points",          5);
  declare("max_cluster_range_span",      0.03);
  declare("min_cluster_angle_width",     0.01);
  declare("max_cluster_angle_width",     0.12);
  
  declare("max_detect_range",            3.0);
  declare("max_fail_count",              5);
  declare("staging_x_offset",           0.8);
  declare("staging_yaw_offset",          3.14159);
  declare("docking_threshold",           0.32);
  declare("use_external_detection_pose", false);
  declare("rotate_to_dock",             false);
  declare("cluster_beam_gap",           3);
  
  // ── PHASE C: Near-range stopping parameters ──
  declare("use_near_range_stop",              true);
  declare("near_range_entry_distance",        0.50);
  declare("near_range_sector_half_angle_deg", 15.0);
  declare("near_range_stop_threshold",        0.10);
  declare("near_range_required_stable_count", 3);
  declare("near_range_statistic",             std::string("median"));

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

  lrf_tilt_alpha_              = angles::from_degrees(get_d("lrf_tilt_alpha_deg"));
  lrf_forward_offset_          = get_d("lrf_forward_offset");
  tape_distance_               = get_d("tape_distance");
  rubber_width_                = get_d("rubber_width");
  reflector_width_             = get_d("reflector_width");
  
  // NEW: Cluster-based detection parameters
  intensity_cluster_threshold_ = static_cast<float>(get_d("intensity_cluster_threshold"));
  min_cluster_points_          = get_i("min_cluster_points");
  max_cluster_range_span_      = get_d("max_cluster_range_span");
  min_cluster_angle_width_     = get_d("min_cluster_angle_width");
  max_cluster_angle_width_     = get_d("max_cluster_angle_width");
  
  max_detect_range_            = get_d("max_detect_range");
  max_fail_count_              = get_i("max_fail_count");
  staging_x_offset_            = get_d("staging_x_offset");
  staging_yaw_offset_          = get_d("staging_yaw_offset");
  docking_threshold_           = get_d("docking_threshold");
  use_external_detection_pose_ = get_b("use_external_detection_pose");
  rotate_to_dock_              = get_b("rotate_to_dock");
  cluster_beam_gap_            = get_i("cluster_beam_gap");

  // ── PHASE C: Load near-range parameters ──
  use_near_range_stop_              = get_b("use_near_range_stop");
  near_range_entry_distance_        = get_d("near_range_entry_distance");
  near_range_sector_half_angle_rad_ = angles::from_degrees(get_d("near_range_sector_half_angle_deg"));
  near_range_stop_threshold_        = get_d("near_range_stop_threshold");
  near_range_required_stable_count_ = get_i("near_range_required_stable_count");
  near_range_statistic_             = node->get_parameter(name_ + ".near_range_statistic").as_string();

  RCLCPP_INFO(node->get_logger(),
    "[%s] Configured: scan=%s base=%s lrf_offset=%.3f tape=%.3f "
    "cluster_thresh=%.0f min_pts=%d max_range=%.1f max_fail=%d "
    "staging_x=%.3f staging_yaw=%.3f use_ext=%s "
    "near_range=%s entry_dist=%.2f sector_deg=%.1f stop_thresh=%.3f stable_cnt=%d stat=%s",
    name_.c_str(), scan_topic_.c_str(), base_frame_.c_str(),
    lrf_forward_offset_, tape_distance_,
    static_cast<double>(intensity_cluster_threshold_), min_cluster_points_,
    max_detect_range_, max_fail_count_,
    staging_x_offset_, staging_yaw_offset_,
    use_external_detection_pose_ ? "true" : "false",
    use_near_range_stop_ ? "enabled" : "disabled",
    near_range_entry_distance_,
    angles::to_degrees(near_range_sector_half_angle_rad_),
    near_range_stop_threshold_,
    near_range_required_stable_count_,
    near_range_statistic_.c_str());
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
    refined_pose_latched_ = geometry_msgs::msg::PoseStamped{};
    has_refined_pose_latch_ = false;
    near_range_stable_count_ = 0;  // NEW: reset near-range counter
  }
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    last_scan_.reset();  // NEW: clear scan state
  }
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LidarIntensityDock::scanCallback, this, std::placeholders::_1));
  detected_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/detected_dock_pose", rclcpp::SystemDefaultsQoS());
  dock_pose_odom_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/dock_pose_in_odom", rclcpp::SystemDefaultsQoS());
  dock_distance_pub_ = node->create_publisher<std_msgs::msg::Float32>(
    "/dock_distance", rclcpp::SystemDefaultsQoS());
  dock_near_range_pub_ = node->create_publisher<std_msgs::msg::Float32>(  // NEW: near-range debug topic
    "/dock_near_range", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(node->get_logger(), "[%s] Active - listening to %s",
    name_.c_str(), scan_topic_.c_str());
}

void LidarIntensityDock::deactivate() { 
  scan_sub_.reset(); 
  detected_pose_pub_.reset(); 
  dock_pose_odom_pub_.reset(); 
  dock_distance_pub_.reset();
  dock_near_range_pub_.reset();  // NEW: cleanup near-range publisher
}

// ─────────────────────────────────────────────
// ChargingDock interface
// ─────────────────────────────────────────────
// Vi tri Pre-dock before detecting tape 

geometry_msgs::msg::PoseStamped
LidarIntensityDock::getStagingPose(
  const geometry_msgs::msg::Pose & dock_pose,
  const std::string & frame)
{
  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = frame;
  auto node = node_.lock();
  staging.header.stamp    = node ? node->now() : rclcpp::Clock().now();
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
  if (!use_external_detection_pose_) {
    // The docking server already transforms dock_pose into fixed_frame (odom)
    // before calling this. Just return true to accept the pose as-is.
    return true;
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);

  if (dock_detected_) {
    try {
      geometry_msgs::msg::PoseStamped last_copy = last_detected_pose_;
      last_copy.header.stamp = rclcpp::Time(0);
      geometry_msgs::msg::PoseStamped pose_in_base;
      tf_->transform(last_copy, pose_in_base, base_frame_, tf2::durationFromSec(0.1));
      refined_pose_latched_   = pose_in_base;
      has_refined_pose_latch_ = true;
      pose = pose_in_base;
      return true;
    } catch (const tf2::TransformException & ex) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN(node->get_logger(),
          "[%s] TF transform failed: %s", name_.c_str(), ex.what());
      }
    }
  }

  if (has_refined_pose_latch_) {
    pose = refined_pose_latched_;
    return true;
  }

  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// isDocked() — PHASE C: Two-mode docking detection
// ─────────────────────────────────────────────────────────────────────────────
// Called repeatedly by the Nav2 docking server after the approach phase
// completes, to decide whether the robot has successfully reached the dock.
//
// Returns true  → docking server advances to the "wait for charging" phase.
// Returns false → server keeps retrying / eventually times out with error 905.
//
// ── WHY THE OLD IMPLEMENTATION FAILS NEAR THE DOCK ──
// The original isDocked() computes distance as:
//   dx = dock_in_odom.pose.position.x - robot_tf.transform.translation.x
//   dy = dock_in_odom.pose.position.y - robot_tf.transform.translation.y
//   dist = hypot(dx, dy)
//
// Where:
//   • dx, dy = 2D offset from robot's current position to the last detected dock pose
//   • Both positions are in the "odom" frame (fixed odometry frame)
//   • dist = 2D Euclidean distance between robot and dock
//
// Problem: When the robot gets very close to the dock:
//   1. The 2-reflector pair often becomes unstable or disappears from the scan
//      (beams too close, geometry breaks down, occlusion, etc.)
//   2. detectReflectors() fails → dock_detected_ becomes false after max_fail_count_ misses
//   3. Code falls back to refined_pose_latched_ (last good pose before detection was lost)
//   4. But refined_pose_latched_ is FROZEN — it doesn't update as robot continues moving
//   5. So dist becomes STALE and may never drop below docking_threshold_
//   6. Result: docking fails with timeout error 905
//
// ── NEW TWO-MODE SOLUTION ──
//
// MODE 1 — REFLECTIVE-POSE MODE (medium range, reflectors visible)
//   • If dock_detected_ == true (live reflector pair detected):
//       → Use last_detected_pose_ (continuously updated)
//   • If dock_detected_ == false but has_refined_pose_latch_ == true:
//       → Use refined_pose_latched_ (last good pose before loss)
//   • Transform pose to odom, compute dist = hypot(dx, dy)
//   • If dist < docking_threshold_ (0.32m default) → return true
//   • If dist < near_range_entry_distance_ (0.50m) and near-range mode enabled:
//       → Switch to MODE 2
//
// MODE 2 — NEAR-RANGE DIRECT-RANGE MODE (very close, reflectors unstable)
//   • Use latest rear lidar scan directly (last_scan_)
//   • Extract ranges from a narrow angular sector around the rear direction
//     (±near_range_sector_half_angle_rad_ around π rad in base_link frame)
//   • Compute robust range estimate using configured statistic:
//       - "min": minimum valid range in sector
//       - "median": median of valid ranges
//       - "trimmed_mean": mean after removing outliers
//   • If range < near_range_stop_threshold_ (0.10m default):
//       → Increment near_range_stable_count_
//       → If count >= near_range_required_stable_count_ (3 cycles) → return true
//   • Else: reset counter
//
// This hybrid approach:
//   • Uses reflective detection at medium range (accurate pose estimation)
//   • Switches to direct range at close range (robust to reflector loss)
//   • Debounces near-range detection to avoid false positives from noise
// ─────────────────────────────────────────────────────────────────────────────

bool LidarIntensityDock::isDocked()
{
  if (!use_external_detection_pose_) {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    // ── Require at least one valid detection before attempting docking check ──
    if (last_detected_pose_.header.stamp.sec == 0) { 
      return false; 
    }

    // ── Pick best available reference pose ──
    geometry_msgs::msg::PoseStamped ref;
    if (dock_detected_) {
      ref = last_detected_pose_;  // Live detection available
    } else if (has_refined_pose_latch_) {
      ref = refined_pose_latched_;  // Fallback to last good pose
    } else {
      return false;  // No pose available
    }

    try {
      // ── Transform dock pose to odom frame ──
      ref.header.stamp = rclcpp::Time(0);  // Use latest available transform
      geometry_msgs::msg::PoseStamped dock_in_odom;
      tf_->transform(ref, dock_in_odom, "odom", tf2::durationFromSec(0.1));

      // ── Get robot position in odom frame ──
      geometry_msgs::msg::TransformStamped robot_tf =
        tf_->lookupTransform("odom", base_frame_, tf2::TimePointZero);

      // ── Compute 2D distance from robot to dock pose ──
      double dx = dock_in_odom.pose.position.x - robot_tf.transform.translation.x;
      double dy = dock_in_odom.pose.position.y - robot_tf.transform.translation.y;
      double dist = std::hypot(dx, dy);

      // ── Publish debug topics ──
      auto node = node_.lock();
      if (node && dock_distance_pub_) {
        std_msgs::msg::Float32 dist_msg;
        dist_msg.data = static_cast<float>(dist);
        dock_distance_pub_->publish(dist_msg);
      }
      if (node && dock_pose_odom_pub_) {
        dock_pose_odom_pub_->publish(dock_in_odom);
      }

      // ── MODE 1: REFLECTIVE-POSE MODE ──
      // Check if standard pose-based distance is below threshold
      if (dist < docking_threshold_) {
        if (node) {
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
            "[%s] isDocked: REFLECTIVE-POSE mode → dist=%.3fm < thresh=%.3fm → DOCKED",
            name_.c_str(), dist, docking_threshold_);
        }
        return true;
      }

      // ── MODE 2: NEAR-RANGE DIRECT-RANGE MODE ──
      // If enabled and robot is close enough, switch to direct range measurement
      if (use_near_range_stop_ && dist < near_range_entry_distance_) {
        sensor_msgs::msg::LaserScan::SharedPtr scan_copy;
        {
          std::lock_guard<std::mutex> scan_lock(scan_mutex_);
          scan_copy = last_scan_;
        }

        if (scan_copy) {
          double near_range = 0.0;
          if (computeNearRangeEstimate(*scan_copy, near_range)) {
            // Publish near-range debug topic
            if (node && dock_near_range_pub_) {
              std_msgs::msg::Float32 nr_msg;
              nr_msg.data = static_cast<float>(near_range);
              dock_near_range_pub_->publish(nr_msg);
            }

            // Check if near-range is below stopping threshold
            if (near_range < near_range_stop_threshold_) {
              near_range_stable_count_++;
              if (near_range_stable_count_ >= near_range_required_stable_count_) {
                if (node) {
                  RCLCPP_INFO(node->get_logger(),
                    "[%s] isDocked: NEAR-RANGE mode → range=%.3fm < thresh=%.3fm "
                    "for %d cycles → DOCKED",
                    name_.c_str(), near_range, near_range_stop_threshold_,
                    near_range_stable_count_);
                }
                return true;
              } else {
                if (node) {
                  RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                    "[%s] isDocked: NEAR-RANGE mode → range=%.3fm < thresh=%.3fm "
                    "(%d/%d cycles)",
                    name_.c_str(), near_range, near_range_stop_threshold_,
                    near_range_stable_count_, near_range_required_stable_count_);
                }
              }
            } else {
              // Range above threshold → reset counter
              if (near_range_stable_count_ > 0) {
                if (node) {
                  RCLCPP_INFO(node->get_logger(),
                    "[%s] isDocked: NEAR-RANGE mode → range=%.3fm >= thresh=%.3fm "
                    "→ reset counter",
                    name_.c_str(), near_range, near_range_stop_threshold_);
                }
                near_range_stable_count_ = 0;
              }
            }
          } else {
            // Failed to compute near-range estimate
            if (node) {
              RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                "[%s] isDocked: NEAR-RANGE mode active but failed to compute range estimate",
                name_.c_str());
            }
          }
        }
      }

      return false;  // Neither mode satisfied docking condition

    } catch (const tf2::TransformException & ex) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN(node->get_logger(),
          "[%s] isDocked() TF failed: %s", name_.c_str(), ex.what());
      }
      return false;
    }
  }

  // ── External detection mode (unchanged) ──
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!dock_detected_) { return false; }
  try {
    geometry_msgs::msg::PoseStamped pose_base;
    tf_->transform(last_detected_pose_, pose_base, base_frame_,
                   tf2::durationFromSec(0.1));
    double dist = std::hypot(pose_base.pose.position.x, pose_base.pose.position.y);
    return dist < docking_threshold_;
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "[%s] isDocked() external TF failed: %s", name_.c_str(), ex.what());
    }
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
  // ── PHASE C: Store latest scan for near-range processing ──
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    last_scan_ = msg;
  }

  auto reflectors = detectReflectors(*msg);
  auto clusters   = clusterReflectors(reflectors);

  geometry_msgs::msg::PoseStamped detected;
  detected.header = msg->header;

  bool found = false;

  if (clusters.size() == 2u) {
    const ReflectorCluster & right =
      (clusters[0].peak_idx < clusters[1].peak_idx)
      ? clusters[0] : clusters[1];
    const ReflectorCluster & left =
      (clusters[0].peak_idx < clusters[1].peak_idx)
      ? clusters[1] : clusters[0];

    found = computeDockPose(left, right, tape_distance_, lrf_forward_offset_, detected);
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (found) {
    miss_count_        = 0;
    dock_detected_     = true;
    last_detected_pose_ = detected;
    detected_pose_pub_->publish(detected);
  } else {
    // Debounce: only clear detection after max_fail_count_ consecutive misses
    if (++miss_count_ > max_fail_count_) {
      dock_detected_ = false;
    }
  }
}

// ─────────────────────────────────────────────
// detectReflectors() - NEW: Cluster-based detection
// ─────────────────────────────────────────────

std::vector<LidarIntensityDock::Reflector>
LidarIntensityDock::detectReflectors(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Reflector> result;

  const size_t N = scan.ranges.size();
  if (scan.intensities.size() != N) {
    return result;
  }

  int i = 0;

  while (i < static_cast<int>(N)) {

    // ===== Skip until a bright point is found =====
    while (i < static_cast<int>(N) &&
           scan.intensities[i] < intensity_cluster_threshold_) {
      i++;
    }

    if (i >= static_cast<int>(N)) {
      break;
    }

    // ===== Start cluster =====
    int cluster_start = i;

    std::vector<int> indices;

    while (i < static_cast<int>(N) &&
           scan.intensities[i] >= intensity_cluster_threshold_) {
      indices.push_back(i);
      i++;
    }

    int cluster_end = i - 1;

    // ===== Check cluster size =====
    if (static_cast<int>(indices.size()) < min_cluster_points_) {
      continue;
    }

    // ===== Compute cluster properties =====
    double sum_I = 0.0;
    double sum_idx_I = 0.0;
    double sum_range_I = 0.0;

    double min_range = 1e9;
    double max_range = -1e9;

    float strongest_I = 0.0f;

    for (int idx : indices) {
      float I = scan.intensities[idx];
      float r = scan.ranges[idx];

      if (!std::isfinite(r) ||
          r < scan.range_min ||
          r > scan.range_max) {
        continue;
      }

      sum_I += I;
      sum_idx_I += static_cast<double>(idx) * I;
      sum_range_I += static_cast<double>(r) * I;

      min_range = std::min(min_range, static_cast<double>(r));
      max_range = std::max(max_range, static_cast<double>(r));

      strongest_I = std::max(strongest_I, I);
    }

    if (sum_I < 1e-6) {
      continue;
    }

    // ===== Range consistency check =====
    double range_span = max_range - min_range;

    if (range_span > max_cluster_range_span_) {
      continue;
    }

    // ===== Angular width check =====
    double angle_start =
      scan.angle_min + cluster_start * scan.angle_increment;

    double angle_end =
      scan.angle_min + cluster_end * scan.angle_increment;

    double cluster_angle_width =
      std::abs(angle_end - angle_start);

    if (cluster_angle_width < min_cluster_angle_width_ ||
        cluster_angle_width > max_cluster_angle_width_) {
      continue;
    }

    // ===== Weighted center =====
    double center_idx = sum_idx_I / sum_I;
    double center_range = sum_range_I / sum_I;

    // ===== Convert to x,y =====
    double rplidar_angle =
      scan.angle_min + center_idx * scan.angle_increment;

    double ros_angle = M_PI - rplidar_angle;

    while (ros_angle > M_PI) {
      ros_angle -= 2.0 * M_PI;
    }

    while (ros_angle < -M_PI) {
      ros_angle += 2.0 * M_PI;
    }

    double x = center_range * std::cos(ros_angle);
    double y = center_range * std::sin(ros_angle);

    // ===== Create reflector =====
    Reflector ref;
    ref.peak_idx = static_cast<int>(std::round(center_idx));
    ref.valley_l_idx = cluster_start;
    ref.valley_r_idx = cluster_end;
    ref.I_peak = strongest_I;
    ref.I_valley = 0.0f;
    ref.L_peak = center_range;
    ref.beta = 0.0;   // optional for now
    ref.x = x;
    ref.y = y;

    result.push_back(ref);
  }

  return result;
}

// ─────────────────────────────────────────────
// clusterReflectors()
// ─────────────────────────────────────────────
// Groups adjacent Reflector peaks (within cluster_beam_gap_ beam indices) into
// clusters. Each cluster's centroid (x, y) and average beta are used downstream
// instead of a single peak beam, giving a more stable tape centre estimate.

std::vector<LidarIntensityDock::ReflectorCluster>
LidarIntensityDock::clusterReflectors(
  const std::vector<Reflector> & reflectors) const
{
  std::vector<ReflectorCluster> clusters;
  if (reflectors.empty()) { return clusters; }

  // Sort by beam index so adjacency check is sequential
  std::vector<const Reflector *> sorted;
  sorted.reserve(reflectors.size());
  for (const auto & r : reflectors) { sorted.push_back(&r); }
  std::sort(sorted.begin(), sorted.end(),
    [](const Reflector * a, const Reflector * b) { return a->peak_idx < b->peak_idx; });

  // Greedy merge: if next peak is within cluster_beam_gap_ of current cluster's last peak, merge
  std::vector<std::vector<const Reflector *>> groups;
  groups.push_back({sorted[0]});

  for (size_t i = 1; i < sorted.size(); ++i) {
    int last_idx = groups.back().back()->peak_idx;
    if (sorted[i]->peak_idx - last_idx <= cluster_beam_gap_) {
      groups.back().push_back(sorted[i]);
    } else {
      groups.push_back({sorted[i]});
    }
  }

  // Compute centroid and average beta per group
  for (const auto & group : groups) {
    ReflectorCluster c;
    c.x = 0.0; c.y = 0.0; c.beta = 0.0;
    for (const auto * r : group) {
      c.x    += r->x;
      c.y    += r->y;
      c.beta += r->beta;
    }
    double n = static_cast<double>(group.size());
    c.x    /= n;
    c.y    /= n;
    c.beta /= n;
    c.peak_idx = group[group.size() / 2]->peak_idx;  // median index as representative
    clusters.push_back(c);
  }

  return clusters;
}

// ─────────────────────────────────────────────
// Inception angle - Eq.(3a)
// ─────────────────────────────────────────────
// Computes beta: the angle of incidence of the LiDAR beam on the reflective tape surface.
//
// Geometry: two adjacent beams i and j=(i-1) hit the tape surface.
//   Li        = range of beam i (the peak beam, hitting tape center)
//   Lj        = range of beam j = i-1 (the beam just before the peak)
//   theta_rad = angular step between consecutive beams (scan.angle_increment)
//
// Using the law of cosines on the triangle formed by the sensor origin and
// the two beam endpoints on the tape surface, beta is derived as:
//
//   beta = atan( Lj * sin(theta) / (Lj * cos(theta) - Li) )
//
// beta represents how obliquely the beam strikes the tape:
//   - Large |beta| → beam hits tape at a steep angle (good detection)
//   - |beta| < 0.05 rad → beam nearly parallel to tape surface → rejecte

double LidarIntensityDock::computeInceptionAngle(
  double Li, double Lj, double theta_rad) const
{
  double denom = Lj * std::cos(theta_rad) - Li;
  if (std::abs(denom) < 1e-6) { return 0.0; }
  return std::atan(Lj * std::sin(theta_rad) / denom);
}

// ─────────────────────────────────────────────
// Dock pose - NEW: Perpendicular-based computation
// ─────────────────────────────────────────────

bool LidarIntensityDock::computeDockPose(
  const ReflectorCluster & left_tape,
  const ReflectorCluster & right_tape,
  double tape_dist,
  double dock_offset,
  geometry_msgs::msg::PoseStamped & pose_out) const
{
  // 1) Check tape spacing
  double dx = left_tape.x - right_tape.x;
  double dy = left_tape.y - right_tape.y;
  double measured_dist = std::hypot(dx, dy);

  if (std::abs(measured_dist - tape_dist) > tape_dist * 0.2) {
    return false;  // geometry inconsistent — likely false positive pair
  }

  // 2) Midpoint of two tapes
  double mx = 0.5 * (left_tape.x + right_tape.x);
  double my = 0.5 * (left_tape.y + right_tape.y);

  // 3) Tangent direction along tape line
  double tx = dx / measured_dist;
  double ty = dy / measured_dist;

  // 4) Two candidate normals
  double n1x = -ty;
  double n1y =  tx;

  double n2x =  ty;
  double n2y = -tx;

  // 5) Choose the normal pointing toward lidar/robot
  // Lidar origin is (0,0), so vector from midpoint to lidar is (-mx, -my)
  double vx = -mx;
  double vy = -my;

  double dot1 = n1x * vx + n1y * vy;
  double dot2 = n2x * vx + n2y * vy;

  double nx, ny;
  if (dot1 >= dot2) {
    nx = n1x;
    ny = n1y;
  } else {
    nx = n2x;
    ny = n2y;
  }
  double yaw = std::atan2(ny, nx);

  // RPLidar frame correction:
  // 1) invert x
  // 2) rotate orientation by 180 deg around z
  yaw += M_PI;

  // normalize yaw to [-pi, pi]
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }

  double d = std::abs(dock_offset);

  double px = mx - d * nx;
  double py = my - d * ny;

  // apply x sign correction
  pose_out.pose.position.x = -px + 0.51;
  pose_out.pose.position.y = py;
  pose_out.pose.position.z = 0.0;

  pose_out.pose.orientation.x = 0.0;
  pose_out.pose.orientation.y = 0.0;
  pose_out.pose.orientation.z = std::sin(yaw / 2.0);
  pose_out.pose.orientation.w = std::cos(yaw / 2.0);

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// computeNearRangeEstimate() — PHASE C: Direct range measurement for near-range stopping
// ─────────────────────────────────────────────────────────────────────────────
// Computes a robust range estimate from a narrow angular sector of the rear lidar scan.
//
// Purpose:
//   When the robot is very close to the dock, the 2-reflector pair detection becomes
//   unreliable (geometry breaks down, beams too close, occlusion). This method provides
//   a fallback by directly measuring the distance to the dock using raw scan ranges.
//
// Method:
//   1. Define a narrow angular sector around the REAR direction (π rad in base_link frame)
//      Width: ±near_range_sector_half_angle_rad_ (default ±15° = ±0.26 rad)
//   2. Extract all valid ranges within this sector from the scan
//   3. Compute a robust statistic (min, median, or trimmed_mean) to filter noise/outliers
//   4. Return the estimate via range_out
//
// Parameters:
//   scan      - LaserScan message (typically from /scan_rear_lidar_filter)
//   range_out - Output: computed range estimate (m)
//
// Returns:
//   true  - Valid estimate computed
//   false - Failed (no valid ranges in sector, or other error)
//
// Frame convention:
//   • Rear direction in base_link: π rad (180°)
//   • Scan angles are in the lidar's own frame (need conversion)
//   • For rear lidar mounted backward: scan angle 0 points rearward
//     → sector center is at scan angle ≈ 0 (depends on lidar mounting)
//
// Note: This assumes the rear lidar is mounted such that its 0° direction
//       points toward the rear of the robot. Adjust sector center if needed.
// ─────────────────────────────────────────────────────────────────────────────
bool LidarIntensityDock::computeNearRangeEstimate(
  const sensor_msgs::msg::LaserScan & scan,
  double & range_out) const
{
  const size_t N = scan.ranges.size();
  if (N == 0) { return false; }

  // ── Define angular sector around rear direction ──
  // For a rear-mounted lidar with angle_min ≈ 0 pointing rearward:
  //   • Sector center: 0 rad (or π rad if lidar frame is flipped)
  //   • Sector bounds: [-near_range_sector_half_angle_rad_, +near_range_sector_half_angle_rad_]
  //
  // Adjust this if your lidar mounting is different.
  // Here we assume the rear lidar's 0° direction points directly backward.
  const double sector_center = 0.0;  // Rear direction in lidar frame
  const double sector_min = sector_center - near_range_sector_half_angle_rad_;
  const double sector_max = sector_center + near_range_sector_half_angle_rad_;

  // ── Collect valid ranges within sector ──
  std::vector<double> valid_ranges;
  valid_ranges.reserve(N / 4);  // Rough estimate for a ±15° sector

  for (size_t i = 0; i < N; ++i) {
    const float r = scan.ranges[i];
    
    // Skip invalid ranges (filtered out, out of bounds, inf, nan)
    if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max) {
      continue;
    }

    // Compute beam angle in lidar frame
    double angle = scan.angle_min + i * scan.angle_increment;
    
    // Normalize angle to [-π, π]
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;

    // Check if beam is within sector
    if (angle >= sector_min && angle <= sector_max) {
      valid_ranges.push_back(static_cast<double>(r));
    }
  }

  // ── Check if we have enough data ──
  if (valid_ranges.empty()) {
    return false;  // No valid ranges in sector
  }

  // ── Compute robust statistic ──
  double estimate = 0.0;

  if (near_range_statistic_ == "min") {
    // Minimum range (most conservative, sensitive to noise)
    estimate = *std::min_element(valid_ranges.begin(), valid_ranges.end());

  } else if (near_range_statistic_ == "median") {
    // Median range (robust to outliers, recommended)
    std::sort(valid_ranges.begin(), valid_ranges.end());
    size_t mid = valid_ranges.size() / 2;
    if (valid_ranges.size() % 2 == 0) {
      estimate = (valid_ranges[mid - 1] + valid_ranges[mid]) / 2.0;
    } else {
      estimate = valid_ranges[mid];
    }

  } else if (near_range_statistic_ == "trimmed_mean") {
    // Trimmed mean: remove top/bottom 10%, average the rest
    if (valid_ranges.size() < 5) {
      // Too few samples for trimming, fall back to mean
      estimate = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0) 
                 / valid_ranges.size();
    } else {
      std::sort(valid_ranges.begin(), valid_ranges.end());
      size_t trim_count = valid_ranges.size() / 10;  // 10% trim
      if (trim_count == 0) trim_count = 1;
      
      double sum = 0.0;
      size_t count = 0;
      for (size_t i = trim_count; i < valid_ranges.size() - trim_count; ++i) {
        sum += valid_ranges[i];
        count++;
      }
      estimate = (count > 0) ? (sum / count) : valid_ranges[valid_ranges.size() / 2];
    }

  } else {
    // Unknown statistic, fall back to median
    std::sort(valid_ranges.begin(), valid_ranges.end());
    size_t mid = valid_ranges.size() / 2;
    estimate = (valid_ranges.size() % 2 == 0)
               ? (valid_ranges[mid - 1] + valid_ranges[mid]) / 2.0
               : valid_ranges[mid];
  }

  range_out = estimate;
  return true;
}

}  // namespace lidar_dock_detector

PLUGINLIB_EXPORT_CLASS(
  lidar_dock_detector::LidarIntensityDock,
  opennav_docking_core::ChargingDock)
