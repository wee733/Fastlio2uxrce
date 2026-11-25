#include <cmath>
#include <chrono>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "input_gate.hpp"
#include <array>
#include <limits>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class FastlioToPx4VisualOdometry : public rclcpp::Node
{
public:
  FastlioToPx4VisualOdometry()
  : Node("fastlio_to_px4_visual_odometry")
  {
    declare_parameter<std::string>("input_odom_topic", "/Odometry");
    declare_parameter<std::string>("output_topic", "/lio/vehicle_visual_odometry");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/fast_lio/bridge_status");
    declare_parameter<bool>("publish_velocity_from_delta", false);
    declare_parameter<bool>("publish_orientation", true);
    declare_parameter<double>("max_sample_age", 0.3);
    declare_parameter<double>("max_future_time", 0.02);
    declare_parameter<bool>("input_world_is_level", false);
    declare_parameter<int>("reset_counter", 0);
    declare_parameter<double>("pos_var_xy", 0.02);
    declare_parameter<double>("pos_var_z", 0.04);
    declare_parameter<double>("rot_var", 0.05);
    declare_parameter<double>("vel_var", 0.10);

    get_parameter("input_odom_topic", input_odom_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("max_sample_age", max_sample_age_);
    get_parameter("max_future_time", max_future_time_);
    get_parameter("input_world_is_level", input_world_is_level_);
    const auto reset = get_parameter("reset_counter").as_int();
    if (reset < 0 || reset > 255) {
      throw std::invalid_argument("reset_counter must be in [0, 255]");
    }
    reset_counter_ = static_cast<uint8_t>(reset);
    get_parameter("publish_velocity_from_delta", publish_velocity_from_delta_);
    get_parameter("publish_orientation", publish_orientation_);

    get_parameter("pos_var_xy", pos_var_xy_);
    get_parameter("pos_var_z", pos_var_z_);
    get_parameter("rot_var", rot_var_);
    get_parameter("vel_var", vel_var_);

    if (!publish_orientation_) {
      throw std::invalid_argument("FRD position requires orientation for EKF frame alignment");
    }
    if (max_sample_age_ <= 0 || max_future_time_ < 0 || pos_var_xy_ <= 0 ||
      pos_var_z_ <= 0 || rot_var_ <= 0 || vel_var_ <= 0)
    {
      throw std::invalid_argument("Time limits and variances must be positive");
    }

    require_inputs_ = declare_parameter<bool>("require_fresh_imu", false);
    if (require_inputs_) {
      gate_ = std::make_unique<InputGate>(steady_now(), 2.0, 0.1, max_sample_age_, 30.0);
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/lio/imu", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr imu) {
          const double age = (get_clock()->now().nanoseconds() -
            rclcpp::Time(imu->header.stamp).nanoseconds()) * 1e-9;
          last_imu_age_ = age;
          if (age >= -max_future_time_ && age <= 0.1) {
            ++imu_count_;
            gate_->imu(steady_now());
          }
        });
      diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        status_topic, 10);
      health_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { health(); });
    }

    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_,
            odom_qos,
            std::bind(&FastlioToPx4VisualOdometry::odomCallback, this, _1));

    visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            output_topic_,
            px4_qos);

    RCLCPP_WARN(
            get_logger(),
            "FAST-LIO bridge: input=%s output=%s; level-world confirmation=%d",
            input_odom_topic_.c_str(), output_topic_.c_str(), input_world_is_level_);

    RCLCPP_WARN(
            get_logger(),
            "Frame conversion: FAST-LIO FLU/camera_init -> PX4 FRD local frame");
  }

  bool failed() const { return gate_ && gate_->failed(); }

private:
  static double steady_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void health() {
    const double now = steady_now();
    const bool ready = gate_->poll(now);
    if (now - last_report_ < 1.0 && !gate_->failed()) { return; }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "FAST-LIO 数据健康";
    status.hardware_id = "fastlio_px4_bridge";
    status.level = gate_->failed() ? 2 : (ready ? 0 : 1);
    status.message = gate_->failed() ? gate_->reason() :
      (ready ? "Ready: fresh IMU and odometry" : "Warming up; PX4 output blocked");
    auto value = [&status](const char * key, double x) {
      diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = std::to_string(x);
      status.values.push_back(kv);
    };
    if (last_report_ > 0) {
      value("imu_hz", (imu_count_ - previous_imu_) / (now - last_report_));
      value("published_hz", (published_ - previous_published_) / (now - last_report_));
    }
    value("odom_age_ms", last_age_ * 1000);
    value("imu_age_ms", last_imu_age_ * 1000);
    value("since_fresh_imu_ms", gate_->since_imu(now) * 1000);
    value("since_fresh_odom_ms", gate_->since_odom(now) * 1000);
    value("odom_received", odom_count_);
    value("odom_published", published_);
    value("odom_rejected_or_warming", odom_count_ - published_);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now(); array.status.push_back(status);
    diagnostic_pub_->publish(array);
    previous_imu_ = imu_count_; previous_published_ = published_; last_report_ = now;
    if (gate_->failed()) {
      RCLCPP_ERROR(get_logger(),
        "%s (time since last fresh input at failure: IMU %.1f ms, odometry %.1f ms)",
        status.message.c_str(), gate_->since_imu(now) * 1000, gate_->since_odom(now) * 1000);
      rclcpp::shutdown(); // launch shuts down the entire estimator, discarding old queues.
    }
  }
  bool require_inputs_{false};
  std::unique_ptr<InputGate> gate_;
  uint64_t imu_count_{0}, odom_count_{0}, published_{0}, previous_imu_{0}, previous_published_{0};
  double last_age_{std::numeric_limits<double>::quiet_NaN()};
  double last_imu_age_{std::numeric_limits<double>::quiet_NaN()};
  double last_report_{0};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  struct QuatXYZW
  {
    double x;
    double y;
    double z;
    double w;
  };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;

  std::string input_odom_topic_ = "/Odometry";

  std::string output_topic_;
  double max_sample_age_ = 0.3;
  double max_future_time_ = 0.02;
  bool input_world_is_level_ = false;
  uint8_t reset_counter_ = 0;
  int64_t last_sample_us_ = 0;

  bool publish_velocity_from_delta_ = false;
  bool publish_orientation_ = true;
  bool has_prev_ = false;
  rclcpp::Time prev_stamp_;
  std::array<float, 3> prev_pos_frd_{0.0f, 0.0f, 0.0f};

  double pos_var_xy_ = 0.02;
  double pos_var_z_ = 0.04;
  double rot_var_ = 0.05;
  double vel_var_ = 0.10;

  uint64_t nowRosTimeUs()
  {
    const int64_t now_us =
      static_cast<int64_t>(this->get_clock()->now().nanoseconds() / 1000);

    return static_cast<uint64_t>(now_us);
  }

  uint64_t rosStampTimeUs(const builtin_interfaces::msg::Time & stamp)
  {
    const int64_t stamp_us =
      static_cast<int64_t>(stamp.sec) * 1000000LL +
      static_cast<int64_t>(stamp.nanosec) / 1000LL;

    return static_cast<uint64_t>(stamp_us);
  }

  static QuatXYZW multiply(const QuatXYZW & a, const QuatXYZW & b)
  {
    QuatXYZW r;

    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

    return r;
  }

  static QuatXYZW conjugate(const QuatXYZW & q)
  {
    return {-q.x, -q.y, -q.z, q.w};
  }

  static QuatXYZW normalize(const QuatXYZW & q)
  {
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    if (n < 1e-9) {
      return {0.0, 0.0, 0.0, 1.0};
    }

    return {q.x / n, q.y / n, q.z / n, q.w / n};
  }

    /*
     * FAST-LIO 常见输出：
     *   camera_init / body 近似 ROS FLU：
     *   x 前，y 左，z 上
     *
     * PX4 外部里程计这里使用 FRD：
     *   x 前，y 右，z 下
     *
     * 位置转换：
     *   x_frd =  x_flu
     *   y_frd = -y_flu
     *   z_frd = -z_flu
     */
  static std::array<float, 3> positionFluToFrd(double x, double y, double z)
  {
    return {
      static_cast<float>(x),
      static_cast<float>(-y),
      static_cast<float>(-z)
    };
  }

    /*
     * 姿态转换：
     * 源：body_FLU 在 world_FLU 下的姿态
     * 目标：body_FRD 在 world_FRD 下的姿态
     *
     * 两边都绕 x 轴翻转 180°：
     *   R_target = C * R_source * C^-1
     */
  static QuatXYZW quatFluToFrd(const geometry_msgs::msg::Quaternion & q_ros)
  {
    QuatXYZW q_src{
      q_ros.x,
      q_ros.y,
      q_ros.z,
      q_ros.w
    };

    q_src = normalize(q_src);

    const QuatXYZW q_flip_x{1.0, 0.0, 0.0, 0.0};
    const QuatXYZW q_target =
      multiply(multiply(q_flip_x, q_src), conjugate(q_flip_x));

    return normalize(q_target);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
        // camera_init must be gravity-aligned, and body must be the FC IMU in FLU.
    if (!input_world_is_level_ || odom->header.frame_id != "camera_init" ||
      odom->child_frame_id != "body")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Reject odometry: confirm level camera_init and body=FC IMU FLU");
      return;
    }
    const int64_t sample_us = static_cast<int64_t>(odom->header.stamp.sec) * 1000000LL +
      odom->header.stamp.nanosec / 1000;
    const auto now_us = nowRosTimeUs();
    const double age = (static_cast<int64_t>(now_us) - sample_us) * 1e-6;
    ++odom_count_;
    last_age_ = age;
    if (sample_us <= 0 || sample_us <= last_sample_us_ ||
      age > max_sample_age_ || age < -max_future_time_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Reject stale/future/out-of-order odometry, age=%.3f", age);
      return;
    }
    const auto & p = odom->pose.pose.position;
    const auto & q = odom->pose.pose.orientation;

    if (!std::isfinite(p.x) ||
      !std::isfinite(p.y) ||
      !std::isfinite(p.z))
    {
      RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Received non-finite FAST-LIO position. Skip.");
      return;
    }

    if (publish_orientation_ &&
      (!std::isfinite(q.x) ||
      !std::isfinite(q.y) ||
      !std::isfinite(q.z) ||
      !std::isfinite(q.w)))
    {
      RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Received non-finite FAST-LIO orientation. Skip.");
      return;
    }
    const double qnorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (std::abs(qnorm - 1.0) > 0.05) {
      return;
    }
    if (gate_) {
      const double steady = steady_now();
      gate_->odom(steady);
      if (!gate_->poll(steady)) { return; }
    }
    px4_msgs::msg::VehicleOdometry msg{};

        // UXRCE_DDS_SYNCT=1 converts both fields on the PX4 side. Do not add offsets.
    msg.timestamp = now_us;
    msg.timestamp_sample = rosStampTimeUs(odom->header.stamp);

    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

    const auto pos_frd = positionFluToFrd(p.x, p.y, p.z);
    msg.position = pos_frd;

    if (publish_orientation_) {
      const auto q_frd = quatFluToFrd(q);

      msg.q = {
        static_cast<float>(q_frd.w),
        static_cast<float>(q_frd.x),
        static_cast<float>(q_frd.y),
        static_cast<float>(q_frd.z)
      };

      msg.orientation_variance = {
        static_cast<float>(rot_var_),
        static_cast<float>(rot_var_),
        static_cast<float>(rot_var_)
      };
    } else {
      const float nan = std::numeric_limits<float>::quiet_NaN();

      msg.q = {nan, nan, nan, nan};
      msg.orientation_variance = {nan, nan, nan};
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();

    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;

    if (publish_velocity_from_delta_ && has_prev_) {
      const rclcpp::Time current_stamp(odom->header.stamp);
      const double dt = (current_stamp - prev_stamp_).seconds();

      if (dt > 1e-3 && dt < 0.5) {
        msg.velocity = {
          static_cast<float>((pos_frd[0] - prev_pos_frd_[0]) / dt),
          static_cast<float>((pos_frd[1] - prev_pos_frd_[1]) / dt),
          static_cast<float>((pos_frd[2] - prev_pos_frd_[2]) / dt)
        };

        msg.velocity_variance = {
          static_cast<float>(vel_var_),
          static_cast<float>(vel_var_),
          static_cast<float>(vel_var_)
        };
      } else {
        msg.velocity = {nan, nan, nan};
        msg.velocity_variance = {nan, nan, nan};
      }
    } else {
      msg.velocity = {nan, nan, nan};
      msg.velocity_variance = {nan, nan, nan};
    }

    msg.angular_velocity = {nan, nan, nan};

    msg.position_variance = {
      static_cast<float>(pos_var_xy_),
      static_cast<float>(pos_var_xy_),
      static_cast<float>(pos_var_z_)
    };

    msg.reset_counter = reset_counter_;
    msg.quality = 0;      // Unknown: FAST-LIO does not provide a calibrated quality score.

    visual_odom_pub_->publish(msg);
    ++published_;

    prev_stamp_ = rclcpp::Time(odom->header.stamp);
    prev_pos_frd_ = pos_frd;
    has_prev_ = true;
    last_sample_us_ = sample_us;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FastlioToPx4VisualOdometry>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return node->failed() ? 2 : 0;
}
