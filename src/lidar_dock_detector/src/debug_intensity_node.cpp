#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>

class DebugIntensityNode : public rclcpp::Node
{
public:
  DebugIntensityNode() : Node("debug_intensity_node")
  {
    // Parameters
    this->declare_parameter("scan_topic",   "/scan_front_filter");
    this->declare_parameter("i_peak",       46.0);
    this->declare_parameter("i_valley",     60.0);
    this->declare_parameter("search_range", 15);

    scan_topic_   = this->get_parameter("scan_topic").as_string();
    i_peak_       = static_cast<float>(
                      this->get_parameter("i_peak").as_double());
    i_valley_     = static_cast<float>(
                      this->get_parameter("i_valley").as_double());
    search_range_ = static_cast<int>(
                      this->get_parameter("search_range").as_int());

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DebugIntensityNode::scanCallback,
                this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Debug node started!\n"
      "  topic=%s\n"
      "  i_peak=%.0f  i_valley=%.0f  search=%d",
      scan_topic_.c_str(),
      static_cast<double>(i_peak_),
      static_cast<double>(i_valley_),
      search_range_);
  }

private:
  void scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->intensities.size() != msg->ranges.size()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(),
        *this->get_clock(), 2000,
        "No intensity data!");
      return;
    }

    const size_t N     = msg->ranges.size();
    const double theta = msg->angle_increment;

    // ── Tìm tất cả peaks ─────────────────────
    struct Peak {
      int   idx;
      float intensity;
      float range;
      float valley_l;
      float valley_r;
      double angle_deg;
      double x, y;
    };

    std::vector<Peak> peaks;
    const int margin = search_range_ + 2;

    for (int i = margin; i < static_cast<int>(N) - margin; ++i)
    {
      float I_i = msg->intensities[i];
      if (I_i < i_peak_) { continue; }

      // Local max?
      bool is_max = true;
      for (int k = i - 1;
           k >= std::max(0, i - search_range_); --k) {
        if (msg->intensities[k] > I_i) {
          is_max = false; break;
        }
      }
      if (!is_max) { continue; }
      for (int k = i + 1;
           k <= std::min((int)N-1, i+search_range_); ++k) {
        if (msg->intensities[k] > I_i) {
          is_max = false; break;
        }
      }
      if (!is_max) { continue; }

      // Valley trái
      float vl = msg->intensities[i-1];
      for (int k = i-2;
           k >= std::max(0, i-search_range_); --k) {
        if (msg->intensities[k] < vl) vl = msg->intensities[k];
      }

      // Valley phải
      float vr = msg->intensities[i+1];
      for (int k = i+2;
           k <= std::min((int)N-1, i+search_range_); ++k) {
        if (msg->intensities[k] < vr) vr = msg->intensities[k];
      }

      // Range hợp lệ?
      float r = msg->ranges[i];
      if (!std::isfinite(r) ||
          r < msg->range_min ||
          r > msg->range_max) { continue; }

      // Tính góc và vị trí
      double rplidar_angle = msg->angle_min + i * theta;
      double ros_angle = M_PI - rplidar_angle;
      while (ros_angle >  M_PI) ros_angle -= 2*M_PI;
      while (ros_angle < -M_PI) ros_angle += 2*M_PI;

      Peak p;
      p.idx        = i;
      p.intensity  = I_i;
      p.range      = r;
      p.valley_l   = vl;
      p.valley_r   = vr;
      p.angle_deg  = rplidar_angle * 180.0 / M_PI;
      p.x          = r * std::cos(ros_angle);
      p.y          = r * std::sin(ros_angle);

      peaks.push_back(p);
      i += search_range_;
    }

    // ── Print kết quả ─────────────────────────
    printf("\033[2J\033[H");
    fflush(stdout);
    RCLCPP_INFO(this->get_logger(),
      "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    if (peaks.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "Không detect được tape nào!"
        "\n  → i_peak=%.0f quá cao? "
        "Peak cao nhất trong scan = %.0f",
        static_cast<double>(i_peak_),
        static_cast<double>(
          *std::max_element(
            msg->intensities.begin(),
            msg->intensities.end())));
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Detect được %zu peak(s):", peaks.size());

      for (size_t i = 0; i < peaks.size(); ++i) {
        auto & p = peaks[i];
        bool valley_ok = (p.valley_l <= i_valley_ &&
                          p.valley_r <= i_valley_);

        RCLCPP_INFO(this->get_logger(),
          "  Peak %zu:"
          "\n    beam=%d  angle=%.1f°"
          "\n    intensity=%.0f"
          "\n    range=%.3fm"
          "\n    valley_L=%.0f  valley_R=%.0f  %s"
          "\n    pos: x=%.3fm  y=%.3fm",
          i+1,
          p.idx,
          p.angle_deg,
          static_cast<double>(p.intensity),
          static_cast<double>(p.range),
          static_cast<double>(p.valley_l),
          static_cast<double>(p.valley_r),
          valley_ok ? "✅ VALID" : "❌ VALLEY TOO HIGH",
          p.x, p.y);
      }

      // Nếu đúng 2 peak → tính khoảng cách
      if (peaks.size() == 2) {
        double dx = peaks[0].x - peaks[1].x;
        double dy = peaks[0].y - peaks[1].y;
        double d  = std::hypot(dx, dy);
        RCLCPP_INFO(this->get_logger(),
          "  → Khoảng cách 2 tape: %.3fm = %.1fcm",
          d, d*100.0);

        double cx = (peaks[0].x + peaks[1].x) / 2.0;
        double cy = (peaks[0].y + peaks[1].y) / 2.0;
        RCLCPP_INFO(this->get_logger(),
          "  → Tâm dock: x=%.3fm  y=%.3fm",
          cx, cy);
      }
    }
  }

  // Members
  std::string scan_topic_;
  float       i_peak_;
  float       i_valley_;
  int         search_range_;

rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<DebugIntensityNode>());
  rclcpp::shutdown();
  return 0;
}