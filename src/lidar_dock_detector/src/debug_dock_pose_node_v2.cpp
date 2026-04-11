#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DebugDockPoseNode : public rclcpp::Node
{
public:
  DebugDockPoseNode()
  : Node("debug_dock_pose_node")
  {
    // ===== same values as your current cpp / yaml =====
    scan_topic_          = "/scan_rear_lidar_filter";
    lrf_forward_offset_  = -0.35;
    tape_distance_       = 0.375;
    rubber_width_        = 0.32;
    reflector_width_     = 0.048;
    i_peak_              = 45.0f;
    i_valley_            = 24.0f;
    valley_search_range_ = 19;
    max_detect_range_    = 3.0;
    intensity_cluster_threshold_ = 45.0f;
    min_cluster_points_ = 5;
    max_cluster_range_span_ = 0.03;   // 3 cm
    min_cluster_angle_width_ = 0.01;  // rad
    max_cluster_angle_width_ = 0.12;  // rad
    // ================================================

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DebugDockPoseNode::scanCallback, this, std::placeholders::_1));

    dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/debug_dock_pose_lidar", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "[debug_dock_pose_node] Listening to %s", scan_topic_.c_str());
  }

private:
  struct Reflector
  {
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

  struct DetectStats
  {
    int reject_peak = 0;
    int reject_local_max = 0;
    int reject_valley = 0;
    int reject_range = 0;
    int reject_beta = 0;
    int accepted = 0;

    float strongest_peak_left = 0.0f;
    float strongest_peak_right = 0.0f;
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    DetectStats stats;
    auto reflectors = detectReflectors(*msg, stats);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[debug_dock_pose_node] reflectors.size() = %zu",
      reflectors.size());

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[DEBUG FILTER] accept=%d peak_rej=%d local_rej=%d valley_rej=%d range_rej=%d beta_rej=%d left_peak=%.1f right_peak=%.1f",
      stats.accepted,
      stats.reject_peak,
      stats.reject_local_max,
      stats.reject_valley,
      stats.reject_range,
      stats.reject_beta,
      stats.strongest_peak_left,
      stats.strongest_peak_right);

    for (size_t k = 0; k < reflectors.size(); ++k) {
      const auto & r = reflectors[k];
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[debug_dock_pose_node] ref[%zu]: idx=%d x=%.3f y=%.3f Ipeak=%.1f Ivalley=%.1f beta=%.3f",
        k, r.peak_idx, r.x, r.y, r.I_peak, r.I_valley, r.beta);
    }

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

    if (found) {
      double yaw = 2.0 * std::atan2(
        detected.pose.orientation.z,
        detected.pose.orientation.w);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[debug_dock_pose_node] DOCK POSE in frame [%s]: x=%.3f y=%.3f yaw=%.3f rad",
        detected.header.frame_id.c_str(),
        detected.pose.position.x,
        detected.pose.position.y,
        yaw);

      dock_pose_pub_->publish(detected);
    }
  }

  // std::vector<Reflector> detectReflectors(
  //   const sensor_msgs::msg::LaserScan & scan,
  //   DetectStats & stats) const
  // {
  //   std::vector<Reflector> result;
  //   const size_t N = scan.ranges.size();

  //   if (scan.intensities.size() != N) {
  //     return result;
  //   }

  //   const double theta = scan.angle_increment;

  //   // same logic as your cpp
  //   const double max_range_geom =
  //     std::min(rubber_width_, reflector_width_) / (2.0 * std::sin(theta / 2.0));
  //   const double max_range = std::min(max_range_geom, max_detect_range_);

  //   const int margin = valley_search_range_ + 2;

  //   for (int i = margin; i < static_cast<int>(N) - margin; ++i) {
  //     if (scan.ranges[i] >= scan.range_max) {
  //       continue;
  //     }

  //     const float I_i = scan.intensities[i];

  //     // Debug strongest raw peaks on left / right side
  //     {
  //       double dbg_angle = scan.angle_min + i * scan.angle_increment;
  //       double dbg_ros_angle = M_PI - dbg_angle;

  //       while (dbg_ros_angle > M_PI) {
  //         dbg_ros_angle -= 2 * M_PI;
  //       }
  //       while (dbg_ros_angle < -M_PI) {
  //         dbg_ros_angle += 2 * M_PI;
  //       }

  //       const float dbg_r = scan.ranges[i];
  //       if (std::isfinite(dbg_r) &&
  //           dbg_r >= scan.range_min &&
  //           dbg_r <= scan.range_max)
  //       {
  //         double dbg_y = static_cast<double>(dbg_r) * std::sin(dbg_ros_angle);

  //         if (dbg_y >= 0.0) {
  //           stats.strongest_peak_left =
  //             std::max(stats.strongest_peak_left, I_i);
  //         } else {
  //           stats.strongest_peak_right =
  //             std::max(stats.strongest_peak_right, I_i);
  //         }
  //       }
  //     }

  //     if (I_i < i_peak_) {
  //       stats.reject_peak++;
  //       continue;
  //     }

  //     bool is_max = true;

  //     for (int k = i - 1; k >= std::max(0, i - valley_search_range_); --k) {
  //       if (scan.intensities[k] > I_i) {
  //         is_max = false;
  //         break;
  //       }
  //     }
  //     if (!is_max) {
  //       stats.reject_local_max++;
  //       continue;
  //     }

  //     for (int k = i + 1; k <= std::min(static_cast<int>(N) - 1, i + valley_search_range_); ++k) {
  //       if (scan.intensities[k] > I_i) {
  //         is_max = false;
  //         break;
  //       }
  //     }
  //     if (!is_max) {
  //       stats.reject_local_max++;
  //       continue;
  //     }

  //     // Left valley
  //     int vl_idx = i - 1;
  //     float vl_val = scan.intensities[vl_idx];
  //     for (int k = i - 2; k >= std::max(0, i - valley_search_range_); --k) {
  //       if (scan.intensities[k] < vl_val) {
  //         vl_val = scan.intensities[k];
  //         vl_idx = k;
  //       }
  //     }

  //     // Right valley
  //     int vr_idx = i + 1;
  //     float vr_val = scan.intensities[vr_idx];
  //     for (int k = i + 2; k <= std::min(static_cast<int>(N) - 1, i + valley_search_range_); ++k) {
  //       if (scan.intensities[k] < vr_val) {
  //         vr_val = scan.intensities[k];
  //         vr_idx = k;
  //       }
  //     }

  //     if (vl_val > i_valley_ || vr_val > i_valley_) {
  //       stats.reject_valley++;
  //       continue;
  //     }

  //     const float r_i = scan.ranges[i];
  //     if (!std::isfinite(r_i) || r_i < scan.range_min || r_i > scan.range_max) {
  //       stats.reject_range++;
  //       continue;
  //     }

  //     if (static_cast<double>(r_i) > max_range) {
  //       stats.reject_range++;
  //       continue;
  //     }

  //     const float r_j = scan.ranges[i - 1];
  //     if (!std::isfinite(r_j) || r_j < scan.range_min || r_j > scan.range_max) {
  //       stats.reject_range++;
  //       continue;
  //     }

  //     double beta = computeInceptionAngle(
  //       static_cast<double>(r_i), static_cast<double>(r_j), theta);

  //     if (std::abs(beta) < 0.05) {
  //       stats.reject_beta++;
  //       continue;
  //     }

  //     double rplidar_angle = scan.angle_min + i * scan.angle_increment;
  //     double ros_angle = M_PI - rplidar_angle;

  //     while (ros_angle > M_PI) {
  //       ros_angle -= 2 * M_PI;
  //     }
  //     while (ros_angle < -M_PI) {
  //       ros_angle += 2 * M_PI;
  //     }

  //     Reflector ref;
  //     ref.peak_idx     = i;
  //     ref.valley_l_idx = vl_idx;
  //     ref.valley_r_idx = vr_idx;
  //     ref.I_peak       = I_i;
  //     ref.I_valley     = std::min(vl_val, vr_val);
  //     ref.L_peak       = static_cast<double>(r_i);
  //     ref.beta         = beta;
  //     ref.x            = static_cast<double>(r_i) * std::cos(ros_angle);
  //     ref.y            = static_cast<double>(r_i) * std::sin(ros_angle);

  //     result.push_back(ref);
  //     stats.accepted++;

  //     // Skip forward to avoid counting the same reflector multiple times
  //     i = vr_idx;
  //   }

  //   return result;
  // }
  std::vector<Reflector> detectReflectors(
  const sensor_msgs::msg::LaserScan & scan,
  DetectStats & stats) const
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
      stats.reject_peak++;
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
      stats.reject_local_max++;
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
        stats.reject_range++;
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
      stats.reject_range++;
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
      stats.reject_valley++;
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

    // ===== Debug left / right peak =====
    if (y >= 0.0) {
      stats.strongest_peak_left =
        std::max(stats.strongest_peak_left, strongest_I);
    } else {
      stats.strongest_peak_right =
        std::max(stats.strongest_peak_right, strongest_I);
    }

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
    stats.accepted++;
  }

  return result;
}

  double computeInceptionAngle(
    double Li, double Lj, double theta_rad) const
  {
    double denom = Lj * std::cos(theta_rad) - Li;
    if (std::abs(denom) < 1e-6) {
      return 0.0;
    }
    return std::atan(Lj * std::sin(theta_rad) / denom);
  }

  // bool computeDockPose(
  //   const Reflector & left_tape,
  //   const Reflector & right_tape,
  //   double tape_dist,
  //   double lrf_offset,
  //   geometry_msgs::msg::PoseStamped & pose_out) const
  // {
  //   double measured_dist = std::hypot(
  //     left_tape.x - right_tape.x,
  //     left_tape.y - right_tape.y);

  //   if (std::abs(measured_dist - tape_dist) > tape_dist * 0.2) {
  //     RCLCPP_WARN(
  //       this->get_logger(),
  //       "[debug_dock_pose_node] Pair rejected: measured_dist=%.3f expected=%.3f",
  //       measured_dist, tape_dist);
  //     return false;
  //   }

  //   double theta_L = std::atan2(left_tape.y, left_tape.x);
  //   double theta_R = std::atan2(right_tape.y, right_tape.x);

  //   double phi_L = M_PI / 2.0 - left_tape.beta - theta_L;
  //   double phi_R = M_PI / 2.0 - right_tape.beta - theta_R;
  //   double phi_m = (phi_L + phi_R) / 2.0;

  //   // same logic as your cpp
  //   pose_out.pose.position.x = (left_tape.x + right_tape.x) / 2.0 + lrf_offset;
  //   pose_out.pose.position.y = (left_tape.y + right_tape.y) / 2.0;
  //   pose_out.pose.position.z = 0.0;

  //   pose_out.pose.orientation.x = 0.0;
  //   pose_out.pose.orientation.y = 0.0;
  //   pose_out.pose.orientation.z = std::sin(phi_m / 2.0);
  //   pose_out.pose.orientation.w = std::cos(phi_m / 2.0);

  //   return true;
  // }
  
bool computeDockPose(
  const Reflector & left_tape,
  const Reflector & right_tape,
  double tape_dist,
  double dock_offset,
  geometry_msgs::msg::PoseStamped & pose_out) const
{
  // 1) Check tape spacing
  double dx = left_tape.x - right_tape.x;
  double dy = left_tape.y - right_tape.y;
  double measured_dist = std::hypot(dx, dy);

  if (std::abs(measured_dist - tape_dist) > tape_dist * 0.2) {
    RCLCPP_WARN(
      this->get_logger(),
      "[debug_dock_pose_node] Pair rejected: measured_dist=%.3f expected=%.3f",
      measured_dist, tape_dist);
    return false;
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

  // 6) Orientation: x-axis of dock pose points outward toward robot
  double yaw = std::atan2(ny, nx);

  // 7) Position:
  // midpoint is on tape line.
  // If dock origin should lie behind the tape line (closer to wall),
  // shift opposite to the outward normal.
  double d = std::abs(dock_offset);

  pose_out.pose.position.x = mx - d * nx;
  pose_out.pose.position.y = my - d * ny;
  pose_out.pose.position.z = 0.0;

  pose_out.pose.orientation.x = 0.0;
  pose_out.pose.orientation.y = 0.0;
  pose_out.pose.orientation.z = std::sin(yaw / 2.0);
  pose_out.pose.orientation.w = std::cos(yaw / 2.0);

  return true;
}

private:
  std::string scan_topic_;
  double lrf_forward_offset_;
  double tape_distance_;
  double rubber_width_;
  double reflector_width_;
  float i_peak_;
  float i_valley_;
  int valley_search_range_;
  double max_detect_range_;
  float intensity_cluster_threshold_;
  int min_cluster_points_;
  double max_cluster_range_span_;
  double min_cluster_angle_width_;
  double max_cluster_angle_width_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugDockPoseNode>());
  rclcpp::shutdown();
  return 0;
}