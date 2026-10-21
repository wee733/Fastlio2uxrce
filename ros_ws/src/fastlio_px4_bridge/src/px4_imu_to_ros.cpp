#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <px4_msgs/msg/sensor_combined.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// SensorCombined is already calibrated and rotated into the vehicle FRD axes.
// Its timestamp is already in Agent/ROS time when UXRCE_DDS_SYNCT=1.
class Px4ImuToRos : public rclcpp::Node
{
public:
  Px4ImuToRos()
  : Node("px4_imu_to_ros")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/fmu/out/sensor_combined");
    const auto output = declare_parameter<std::string>("output_topic", "/lio/imu");
    frame_ = declare_parameter<std::string>("frame_id", "px4_imu_flu");
    max_age_ = declare_parameter<double>("max_sample_age", 0.1);
    max_future_ = declare_parameter<double>("max_future_time", 0.02);
    max_skew_us_ = declare_parameter<int>("max_accel_gyro_skew_us", 1000);
    if (max_age_ <= 0 || max_future_ < 0 || max_skew_us_ < 0) {
      throw std::invalid_argument("Invalid IMU time limits");
    }
    // The inspected FAST-LIO node requests reliable IMU QoS.
    pub_ = create_publisher<sensor_msgs::msg::Imu>(output, rclcpp::QoS(100).reliable());
    sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
      input, rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&Px4ImuToRos::callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "FC IMU FRD -> FLU, %s -> %s", input.c_str(), output.c_str());
  }

private:
  void callback(const px4_msgs::msg::SensorCombined::ConstSharedPtr msg)
  {
    const auto now_us = get_clock()->now().nanoseconds() / 1000;
    const double age = (now_us - static_cast<int64_t>(msg->timestamp)) * 1e-6;
    if (msg->timestamp == 0 || msg->timestamp <= last_stamp_ ||
      age > max_age_ || age < -max_future_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Reject IMU timestamp: age=%.4f s; check XRCE/host clocks", age);
      return;
    }
    // A single sensor_msgs/Imu stamp cannot represent widely separated samples.
    // Do not mask this by stamping the packet with its arrival time.
    const auto relative = static_cast<int64_t>(msg->accelerometer_timestamp_relative);
    if (std::abs(relative) > max_skew_us_ || msg->gyro_integral_dt == 0 ||
      msg->accelerometer_integral_dt == 0 || msg->gyro_clipping || msg->accelerometer_clipping)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Reject invalid/clipped/asynchronous IMU samples");
      return;
    }
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(msg->gyro_rad[i]) || !std::isfinite(msg->accelerometer_m_s2[i])) {
        return;
      }
    }
    sensor_msgs::msg::Imu out;
    out.header.stamp = rclcpp::Time(static_cast<int64_t>(msg->timestamp) * 1000);
    out.header.frame_id = frame_;
    out.orientation.w = 1.0;
    out.orientation_covariance[0] = -1.0;  // No attitude measurement in SensorCombined.
    out.angular_velocity.x = msg->gyro_rad[0];
    out.angular_velocity.y = -msg->gyro_rad[1];
    out.angular_velocity.z = -msg->gyro_rad[2];
    out.linear_acceleration.x = msg->accelerometer_m_s2[0];
    out.linear_acceleration.y = -msg->accelerometer_m_s2[1];
    out.linear_acceleration.z = -msg->accelerometer_m_s2[2];
    // SI units already: rad/s and m/s^2. Retain gravity/specific force.
    // Zero covariance means unknown; noise parameters belong in the LIO config.
    pub_->publish(out);
    last_stamp_ = msg->timestamp;
  }

  std::string frame_;
  double max_age_;
  double max_future_;
  int max_skew_us_;
  uint64_t last_stamp_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4ImuToRos>());
  rclcpp::shutdown();
  return 0;
}
