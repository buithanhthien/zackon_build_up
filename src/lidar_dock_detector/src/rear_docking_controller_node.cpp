#include <cmath>
#include <memory>
#include <string>
#include <limits>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"

class RearDockingControllerNode : public rclcpp::Node
{
public:
  RearDockingControllerNode()
  : Node("rear_docking_controller_node")
  {
    dock_pose_topic_ = declare_parameter<std::string>("dock_pose_topic", "/debug_dock_pose_lidar");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_rear_lidar_filter");
    // cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_test");


    control_frequency_ = declare_parameter<double>("control_frequency", 30.0);

    // Gains
    k_x_ = declare_parameter<double>("k_x", 0.5);
    k_y_ = declare_parameter<double>("k_y", 1.5);
    k_yaw_ = declare_parameter<double>("k_yaw", 2.0);

    // Velocity limits
    v_reverse_max_ = declare_parameter<double>("v_reverse_max", 0.13);
    v_reverse_min_ = declare_parameter<double>("v_reverse_min", 0.1);
    // w_max_ = declare_parameter<double>("w_max", 0.6);
    w_max_ = declare_parameter<double>("w_max", 0.4);

    // Thresholds
    stop_distance_ = declare_parameter<double>("stop_distance", 0.15);
    slowdown_distance_ = declare_parameter<double>("slowdown_distance", 0.40);
    y_align_threshold_ = declare_parameter<double>("y_align_threshold", 0.10);
    yaw_align_threshold_ = declare_parameter<double>("yaw_align_threshold", 0.25);

    // docked_tolerance_ = declare_parameter<double>("docked_tolerance", 0.006);
    docked_tolerance_ = declare_parameter<double>("docked_tolerance", 0.012);

    docked_confirm_count_ = declare_parameter<int>("docked_confirm_count", 5);
    docked_status_topic_ = declare_parameter<std::string>("docked_status_topic", "/rear_docking_status");

    // Pose freshness
    dock_pose_timeout_ = declare_parameter<double>("dock_pose_timeout", 0.30);
    stale_pose_timeout_ = declare_parameter<double>("stale_pose_timeout", 1.0);
    stop_on_lost_pose_ = declare_parameter<bool>("stop_on_lost_pose", true);

    // Safety
    min_range_threshold_ = declare_parameter<double>("min_range_threshold", 0.14);

    // Optional odom freshness
    require_fresh_odom_ = declare_parameter<bool>("require_fresh_odom", false);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.50);

    // PREALIGN thresholds
    prealign_y_threshold_ = declare_parameter<double>("prealign_y_threshold", 0.05);
    prealign_yaw_threshold_ = declare_parameter<double>("prealign_yaw_threshold", 0.15);

    // PREALIGN gains
    // k_prealign_y_ = declare_parameter<double>("k_prealign_y", 2.5);
    k_prealign_y_ = declare_parameter<double>("k_prealign_y", 4.5);

    k_prealign_yaw_ = declare_parameter<double>("k_prealign_yaw", 0.8);

    // PREALIGN velocity limits
    v_prealign_max_ = declare_parameter<double>("v_prealign_max", 0.08);
    w_prealign_max_ = declare_parameter<double>("w_prealign_max", 0.5);

    dock_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      dock_pose_topic_, 10,
      std::bind(&RearDockingControllerNode::dockPoseCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&RearDockingControllerNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&RearDockingControllerNode::scanCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    docked_status_pub_ = create_publisher<std_msgs::msg::Int32>(docked_status_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&RearDockingControllerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "RearDockingControllerNode started");
    RCLCPP_INFO(get_logger(), "dock_pose_topic: %s", dock_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "odom_topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "scan_topic: %s", scan_topic_.c_str());
    RCLCPP_INFO(get_logger(), "docked_status_topic: %s", docked_status_topic_.c_str());
  }

private:
  static double clamp(double v, double lo, double hi)
  { 
    // (-)
    return std::max(lo, std::min(v, hi));
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  void dockPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Neu da dock xong thi bo qua pose moi
    if (state_ == DockState::DOCKED) {
      return;
    }

    last_dock_pose_ = *msg;
    has_dock_pose_ = true;
    last_dock_pose_time_ = now();

    if (state_ == DockState::SEARCH) {
      state_ = DockState::TRACK;

      RCLCPP_INFO(
        get_logger(),
        "[rear_docking_ctrl] Dock pose reacquired -> TRACK MODE");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    has_odom_ = true;
    last_odom_time_ = now();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
    has_scan_ = true;
  }

  double getMinRange() const
  {
    if (!has_scan_ || last_scan_->ranges.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    double min_range = std::numeric_limits<double>::infinity();
    for (const auto& range : last_scan_->ranges) {
      if (std::isfinite(range) && range > 0.0) {
        min_range = std::min(min_range, static_cast<double>(range));
      }
    }
    return min_range;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void publishDockedStatus(int value)
  {
    std_msgs::msg::Int32 msg;
    msg.data = value;
    docked_status_pub_->publish(msg);
  }

  void publishDockedStatusIfChanged(int value)
  {
    if (last_docked_status_value_ != value) {
      publishDockedStatus(value);
      last_docked_status_value_ = value;
    }
  }

  // void publishSearchMotion()
  // {
  //   geometry_msgs::msg::Twist cmd;
  //   cmd.linear.x = 0.0;

  //   const double t = (now() - search_start_time_).seconds();

  //   // alternate left-right every 1 second
  //   const int phase =
  //     static_cast<int>(t / search_switch_period_) % 2;

  //   if (phase == 0) {
  //     cmd.angular.z = search_w_;
  //   } else {
  //     cmd.angular.z = -search_w_;
  //   }

  //   RCLCPP_WARN_THROTTLE(
  //     get_logger(), *get_clock(), 500,
  //     "[rear_docking_ctrl] SEARCH MODE wz=%.3f",
  //     cmd.angular.z);

  //   cmd_pub_->publish(cmd);
  // }
  void publishSearchMotion()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;

    // Nếu chưa có odom thì fallback quay theo 1 chiều
    if (!has_odom_) {
      cmd.angular.z = search_w_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[rear_docking_ctrl] SEARCH MODE (no odom) wz=%.3f",
        cmd.angular.z);
      cmd_pub_->publish(cmd);
      return;
    }

    if (!search_initialized_) {
      resetSearchPattern();
    }

    const double yaw_now = getRobotYawFromOdom();
    double yaw_error = normalizeAngle(search_target_yaw_ - yaw_now);

    // Nếu đã gần target hiện tại thì đổi sang target phía đối diện
    const double yaw_reach_threshold = 3.0 * M_PI / 180.0;   // 3 deg

    if (std::abs(yaw_error) < yaw_reach_threshold) {
      if (search_direction_ == 1) {
        // đã quay xong bên trái -> chuyển sang bên phải cùng biên độ
        search_direction_ = -1;
        search_half_cycle_done_ = true;

        const double amp_rad = search_amplitude_deg_ * M_PI / 180.0;
        search_target_yaw_ = normalizeAngle(search_center_yaw_ - amp_rad);

        RCLCPP_INFO(
          get_logger(),
          "[rear_docking_ctrl] SEARCH switch: RIGHT target=%.3f rad (amp=%.1f deg)",
          search_target_yaw_, search_amplitude_deg_);
      } else {
        // đã quay xong bên phải -> hoàn tất 1 chu kỳ trái+phải
        search_direction_ = 1;

        if (search_half_cycle_done_) {
          search_amplitude_deg_ += search_step_deg_;
          if (search_amplitude_deg_ > search_amplitude_max_deg_) {
            search_amplitude_deg_ = search_amplitude_max_deg_;
          }
        }

        search_half_cycle_done_ = false;

        const double amp_rad = search_amplitude_deg_ * M_PI / 180.0;
        search_target_yaw_ = normalizeAngle(search_center_yaw_ + amp_rad);

        RCLCPP_INFO(
          get_logger(),
          "[rear_docking_ctrl] SEARCH new cycle: LEFT target=%.3f rad (amp=%.1f deg)",
          search_target_yaw_, search_amplitude_deg_);
      }

      yaw_error = normalizeAngle(search_target_yaw_ - yaw_now);
    }

    // điều khiển quay theo sai số góc
    const double k_search_yaw = 1.5;
    cmd.angular.z = clamp(k_search_yaw * yaw_error, -search_w_, search_w_);

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[rear_docking_ctrl] SEARCH MODE yaw=%.3f target=%.3f err=%.3f wz=%.3f amp=%.1f deg",
      yaw_now, search_target_yaw_, yaw_error, cmd.angular.z, search_amplitude_deg_);

    cmd_pub_->publish(cmd);
  }

  double getRobotYawFromOdom() const
  {
    if (!has_odom_) {
      return 0.0;
    }

    const auto & q = last_odom_.pose.pose.orientation;
    return normalizeAngle(2.0 * std::atan2(q.z, q.w));
  }

  void resetSearchPattern()
  {
    search_center_yaw_ = getRobotYawFromOdom();
    search_amplitude_deg_ = 15.0;
    search_direction_ = 1;   // bắt đầu quay trái trước
    search_initialized_ = true;
    search_half_cycle_done_ = false;

    const double amp_rad = search_amplitude_deg_ * M_PI / 180.0;
    search_target_yaw_ = normalizeAngle(search_center_yaw_ + amp_rad);

    RCLCPP_INFO(
      get_logger(),
      "[rear_docking_ctrl] Reset angular search: center=%.3f rad, amp=%.1f deg, first target=%.3f rad",
      search_center_yaw_, search_amplitude_deg_, search_target_yaw_);
  }

  void publishPrealignMotion(double x, double y, double yaw)
  {
    geometry_msgs::msg::Twist cmd;

    // Angular control: prioritize reducing y, then yaw
    double w_cmd = -k_prealign_y_ * y - k_prealign_yaw_ * yaw;
    w_cmd = clamp(w_cmd, -w_prealign_max_, w_prealign_max_);

    // Linear control: small adjustments to help alignment
    double v_cmd = 0.0;

    if (std::abs(y) < 0.08) {
      if (x < -0.55) {
        v_cmd = -0.04;  // reverse slightly if too far
      } else if (x > -0.40) {
        v_cmd = 0.04;   // forward slightly if too close
      } else {
        v_cmd = 0.0;
      }
    } else {
      v_cmd = 0.0;  // prioritize rotation when y is large
    }

    v_cmd = clamp(v_cmd, -v_prealign_max_, v_prealign_max_);

    cmd.linear.x = v_cmd;
    cmd.angular.z = w_cmd;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "[rear_docking_ctrl] PREALIGN pose=(x=%.3f y=%.3f yaw=%.3f) cmd=(vx=%.3f wz=%.3f)",
      x, y, yaw, cmd.linear.x, cmd.angular.z);

    cmd_pub_->publish(cmd);
  }

  void controlLoop()
  {
    // ===============================
    // SAFETY CHECK: Min range
    // ===============================
    const double min_range = getMinRange();
    if (min_range < min_range_threshold_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[rear_docking_ctrl] SAFETY STOP: min_range=%.3fm < threshold=%.3fm",
        min_range, min_range_threshold_);
      publishStop();
      return;
    }

    // ===============================
    // FINAL STATE: DOCKED
    // ===============================
    if (state_ == DockState::DOCKED) {
      publishDockedStatusIfChanged(1);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[rear_docking_ctrl] DOCKED HOLD: robot stopped");

      publishStop();
      return;
    }

    // ===============================
    // SEARCH / TRACK state handling
    // ===============================
    publishDockedStatusIfChanged(0);

    publishDockedStatusIfChanged(0);

    if (!has_dock_pose_) {
      if (state_ != DockState::SEARCH) {
        state_ = DockState::SEARCH;
        search_start_time_ = now();
        search_initialized_ = false;
        //current_search_period_ = search_switch_period_;  // reset về mặc định
        //last_phase_ = -1;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[rear_docking_ctrl] No dock pose -> SEARCH MODE");

      publishSearchMotion();
      return;
    }

    publishDockedStatusIfChanged(0);

    const double pose_age = (now() - last_dock_pose_time_).seconds();
    if (pose_age > dock_pose_timeout_) {
      if (state_ != DockState::SEARCH) {
        state_ = DockState::SEARCH;
        search_start_time_ = now();
        search_initialized_ = false;

        RCLCPP_WARN(
          get_logger(),
          "[rear_docking_ctrl] Dock pose timeout -> SEARCH MODE");
      }

      publishSearchMotion();
      return;
    }

    // valid pose
    state_ = DockState::TRACK;

    publishDockedStatusIfChanged(0);

    if (require_fresh_odom_) {
      if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[rear_docking_ctrl] No odom yet");
        publishStop();
        return;
      }

      const double odom_age = (now() - last_odom_time_).seconds();
      if (odom_age > odom_timeout_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[rear_docking_ctrl] Odom timeout: %.3f s", odom_age);
        publishStop();
        return;
      }
    }

    // +X points toward robot base.
    // Dock behind robot => x < 0 when measurement is correct.
    const double x = last_dock_pose_.pose.position.x;
    const double y = last_dock_pose_.pose.position.y;

    const double yaw = normalizeAngle(
      2.0 * std::atan2(
        last_dock_pose_.pose.orientation.z,
        last_dock_pose_.pose.orientation.w));

    // If dock suddenly appears in front of the rear lidar frame, stop.
    if (x >= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[rear_docking_ctrl] Invalid rear-docking pose: x=%.3f (expected negative)", x);
      publishStop();
      publishDockedStatusIfChanged(0);
      docked_counter_ = 0;
      return;
    }

    // ===============================
    // PREALIGN GATE
    // ===============================
    const bool need_prealign =
      (std::abs(y) > prealign_y_threshold_) ||
      (std::abs(yaw) > prealign_yaw_threshold_);

    if (need_prealign) {
      state_ = DockState::PREALIGN;
    } else {
      if (state_ == DockState::PREALIGN) {
        RCLCPP_INFO(
          get_logger(),
          "[rear_docking_ctrl] PREALIGN completed -> TRACK MODE");
      }
      state_ = DockState::TRACK;
    }

    if (state_ == DockState::PREALIGN) {
      publishDockedStatusIfChanged(0);
      docked_counter_ = 0;
      publishPrealignMotion(x, y, yaw);
      return;
    }

    // ===============================
    // DOCKED ZONE CHECK
    // ===============================
    const double docked_limit = stop_distance_ + docked_tolerance_;

    if (std::abs(x) <= docked_limit) {
      docked_counter_++;

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 200,
        "[rear_docking_ctrl] In docked zone: x=%.3f limit=%.3f count=%d/%d",
        x, docked_limit, docked_counter_, docked_confirm_count_);

      publishStop();
      publishDockedStatusIfChanged(0);

      if (docked_counter_ >= docked_confirm_count_) {
        state_ = DockState::DOCKED;
        publishDockedStatusIfChanged(1);

        RCLCPP_INFO(
          get_logger(),
          "[rear_docking_ctrl] DOCKED CONFIRMED: x=%.3f y=%.3f yaw=%.3f",
          x, y, yaw);

        publishStop();
      }
      return;
    } else {
      docked_counter_ = 0;
    }

    // Reverse speed from longitudinal error
    double v_mag;
    if (std::abs(x) > slowdown_distance_) {
      v_mag = v_reverse_max_;
    } else {
      double ratio = clamp(std::abs(x) / slowdown_distance_, 0.0, 1.0);
      v_mag = v_reverse_min_ + (v_reverse_max_ - v_reverse_min_) * ratio;
    }

    // Angular correction
    double w_cmd = -k_y_ * y + -k_yaw_ * yaw;
    w_cmd = clamp(w_cmd, -w_max_, w_max_);

    // If lateral or yaw error is large, slow down reverse speed
    const bool badly_misaligned =
      (std::abs(y) > y_align_threshold_) ||
      (std::abs(yaw) > yaw_align_threshold_);

    if (badly_misaligned) {
      v_mag *= 0.4;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -v_mag;   // reverse into dock
    cmd.angular.z = w_cmd;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "[rear_docking_ctrl] pose=(x=%.3f y=%.3f yaw=%.3f) cmd=(vx=%.3f wz=%.3f)",
      x, y, yaw, cmd.linear.x, cmd.angular.z);

    publishDockedStatusIfChanged(0);
    cmd_pub_->publish(cmd);
  }

private:
  enum class DockState
  {
    TRACK,
    SEARCH,
    PREALIGN,
    DOCKED
  };

  DockState state_ = DockState::SEARCH;

  // search behavior
  // double search_w_ = 0.15;             // rad/s
  double search_w_ = 0.8;             // rad/s old 0.6
  //double search_switch_period_ = 1.0;  // seconds
  //double current_search_period_ = 1.0;   // giá trị ban đầu
  //int last_phase_ = -1;                  // để detect chuyển phase
  double search_center_yaw_ = 0.0;
  double search_target_yaw_ = 0.0;
  double search_amplitude_deg_ = 15.0;
  double search_step_deg_ = 5.0;
  double search_amplitude_max_deg_ = 45.0;
  int search_direction_ = 1;   // +1: left, -1: right
  bool search_initialized_ = false;
  bool search_half_cycle_done_ = false;
  rclcpp::Time search_start_time_{0, 0, RCL_ROS_TIME};

  std::string dock_pose_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string scan_topic_;
  std::string docked_status_topic_;

  double control_frequency_;

  double k_x_;
  double k_y_;
  double k_yaw_;

  double v_reverse_max_;
  double v_reverse_min_;
  double w_max_;

  double stop_distance_;
  double slowdown_distance_;
  double y_align_threshold_;
  double yaw_align_threshold_;

  double docked_tolerance_;
  int docked_confirm_count_;
  int docked_counter_ = 0;

  double dock_pose_timeout_;
  double stale_pose_timeout_;
  bool stop_on_lost_pose_;

  double min_range_threshold_;

  bool require_fresh_odom_;
  double odom_timeout_;

  double prealign_y_threshold_;
  double prealign_yaw_threshold_;

  double k_prealign_y_;
  double k_prealign_yaw_;

  double v_prealign_max_;
  double w_prealign_max_;

  int last_docked_status_value_ = -1;

  bool has_dock_pose_ = false;
  bool has_odom_ = false;
  bool has_scan_ = false;

  geometry_msgs::msg::PoseStamped last_dock_pose_;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  rclcpp::Time last_dock_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr docked_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearDockingControllerNode>());
  rclcpp::shutdown();
  return 0;
}