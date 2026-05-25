#ifndef LIDAR_DOCK_DETECTOR__LIDAR_INTENSITY_DOCK_HPP_
#define LIDAR_DOCK_DETECTOR__LIDAR_INTENSITY_DOCK_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "opennav_docking_core/charging_dock.hpp"
#include "std_msgs/msg/float32.hpp"

namespace lidar_dock_detector
{

class LidarIntensityDock : public opennav_docking_core::ChargingDock
{
public:
  LidarIntensityDock() = default;
  ~LidarIntensityDock() override = default;

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
  struct Reflector {
    int    peak_idx;
    int    valley_l_idx;
    int    valley_r_idx;
    float  I_peak;
    float  I_valley;
    double L_peak;
    double beta;
    double x;
    double y;
  };

  struct ReflectorCluster {
    double x;
    double y;
    double beta;
    int    peak_idx;
  };

  struct DetectStats {
    int reject_peak = 0;
    int reject_local_max = 0;
    int reject_valley = 0;
    int reject_range = 0;
    int reject_beta = 0;
    int accepted = 0;
    float strongest_peak_left = 0.0f;
    float strongest_peak_right = 0.0f;
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  std::vector<Reflector> detectReflectors(
    const sensor_msgs::msg::LaserScan & scan,
    DetectStats & stats) const;

  std::vector<ReflectorCluster> clusterReflectors(
    const std::vector<Reflector> & reflectors) const;

  double computeInceptionAngle(double Li, double Lj, double theta_rad) const;

  bool computeDockPose(
    const ReflectorCluster & left_tape,
    const ReflectorCluster & right_tape,
    double tape_dist,
    double lrf_offset,
    geometry_msgs::msg::PoseStamped & pose_out) const;

  bool isMeasurementReliable(
    const DetectStats & stats,
    const Reflector & left,
    const Reflector & right) const;

  bool isPoseJumpReasonable(
    const geometry_msgs::msg::PoseStamped & new_pose) const;

  // ── PHASE C: Near-range helper ──
  // Computes robust range estimate from a narrow angular sector of the scan.
  // Returns true if a valid estimate was computed, false otherwise.
  bool computeNearRangeEstimate(
    const sensor_msgs::msg::LaserScan & scan,
    double & range_out) const;

  // Parameters
  std::string name_;                   // Plugin instance name (set by the docking server on configure)
  std::string scan_topic_;             // LaserScan topic to subscribe to (e.g. "/scan_front_filter")
  std::string base_frame_;             // Robot base TF frame (e.g. "base_link")

  double lrf_tilt_alpha_;              // LRF tilt angle (rad) — corrects range if the sensor is mounted at a tilt
  double lrf_forward_offset_;          // Forward offset (m) from base_link origin to the LRF optical centre
  double tape_distance_;               // Expected centre-to-centre distance (m) between the two reflective tapes
  double rubber_width_;                // Width (m) of the rubber/backing strip that holds each tape (used for pose geometry)
  double reflector_width_;             // Width (m) of a single reflective tape strip (used for inception-angle correction)
  
  // NEW: Cluster-based detection parameters
  float  intensity_cluster_threshold_; // Minimum intensity to be part of a bright cluster
  int    min_cluster_points_;          // Minimum number of consecutive bright points to form a valid cluster
  double max_cluster_range_span_;      // Maximum range variation (m) within a cluster
  double min_cluster_angle_width_;     // Minimum angular width (rad) of a valid cluster
  double max_cluster_angle_width_;     // Maximum angular width (rad) of a valid cluster
  
  double max_detect_range_;            // Maximum range (m) beyond which a detected reflector is discarded
  int    max_fail_count_;              // Consecutive scan failures allowed before dock_detected_ is cleared
  double staging_x_offset_;            // Longitudinal offset (m) from dock pose to the staging pose (negative = behind dock)
  double staging_yaw_offset_;          // Yaw offset (rad) added to staging pose orientation (0 = face the dock)
  double docking_threshold_;           // Distance (m) from dock pose at which isDocked() returns true
  bool   use_external_detection_pose_; // If true, skip LiDAR detection and accept pose from an external node
  bool   rotate_to_dock_;              // If true, staging faces away from dock for forward approach; robot rotates and backs in (requires dock_direction: backward)
  std::string dock_direction_;         // Docking direction: "forward" or "backward"
  int    cluster_beam_gap_;            // Max beam index gap to merge adjacent reflector peaks into one cluster

  // ── Quality constraint parameters ──
  float  max_peak_diff_;               // Maximum allowed difference between left/right peak intensities
  double max_pose_jump_dist_;          // Maximum allowed position jump (m) between consecutive poses
  double max_pose_jump_yaw_rad_;       // Maximum allowed yaw jump (rad) between consecutive poses

  // ── PHASE C: Near-range stopping parameters ──
  // These parameters enable robust docking completion when the 2-reflector pair becomes unstable at very close range.
  bool   use_near_range_stop_;              // Enable near-range direct-range stopping mode
  double near_range_entry_distance_;        // Distance (m) below which near-range mode can activate
  double near_range_sector_half_angle_rad_; // Half-width (rad) of angular sector around rear direction for range sampling
  double near_range_stop_threshold_;        // Range (m) below which robot is considered docked in near-range mode
  int    near_range_required_stable_count_; // Consecutive cycles range must stay below threshold to declare docked
  std::string near_range_statistic_;        // Statistic for robust range: "min", "median", "trimmed_mean"

  // ROS handles
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detected_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dock_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dock_near_range_pub_;  // NEW: debug topic for near-range estimate
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // State
  geometry_msgs::msg::PoseStamped last_detected_pose_;
  geometry_msgs::msg::PoseStamped refined_pose_latched_;
  bool       dock_detected_{false};
  bool       has_refined_pose_latch_{false};
  int        miss_count_{0};
  std::mutex pose_mutex_;
  
  // ── Quality-checked pose retention ──
  bool       has_last_valid_pose_{false};
  geometry_msgs::msg::PoseStamped last_valid_pose_;
  
  // ── PHASE C: Near-range stopping state ──
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;  // Most recent scan for near-range range computation
  int        near_range_stable_count_{0};             // Counter for consecutive cycles below threshold
  std::mutex scan_mutex_;                             // Protects last_scan_ access
};

}  // namespace lidar_dock_detector

#endif  // LIDAR_DOCK_DETECTOR__LIDAR_INTENSITY_DOCK_HPP_
