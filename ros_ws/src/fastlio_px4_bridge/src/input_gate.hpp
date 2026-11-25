#pragma once
#include <limits>
#include <string>

// Steady-clock deadlines: wall-clock corrections must not extend a sensor lease.
class InputGate {
public:
  InputGate(double start, double warmup, double imu_timeout, double odom_timeout,
            double startup_timeout)
  : start_(start), warmup_(warmup), imu_timeout_(imu_timeout),
    odom_timeout_(odom_timeout), startup_timeout_(startup_timeout) {}

  void imu(double now) { observe(now, imu_last_, imu_timeout_, "IMU interrupted"); }
  void odom(double now) { observe(now, odom_last_, odom_timeout_, "Odometry interrupted"); }
  bool poll(double now) {
    if (failed()) { return false; }
    const bool imu_fresh = imu_last_ >= 0 && now - imu_last_ <= imu_timeout_;
    const bool odom_fresh = odom_last_ >= 0 && now - odom_last_ <= odom_timeout_;
    if (!imu_fresh || !odom_fresh) {
      stable_since_ = -1;
      if (ready_) {
        if (imu_fresh) {
          fail(now, "Odometry interrupted");
        } else if (odom_fresh) {
          fail(now, "IMU interrupted");
        } else {
          fail(now, "Input stream stale; restart LIO on the ground");
        }
        return false;
      }
    } else if (stable_since_ < 0) {
      stable_since_ = now;
    } else if (now - stable_since_ >= warmup_) {
      ready_ = true;
    }
    if (!ready_ && now - start_ >= startup_timeout_) {
      fail(now, "No stable IMU/odometry before startup timeout; check timesync/PTP");
    }
    return ready_ && !failed();
  }
  bool failed() const { return failure_time_ >= 0; }
  const std::string & reason() const { return reason_; }
  double since_imu(double now) const { return elapsed(now, imu_last_); }
  double since_odom(double now) const { return elapsed(now, odom_last_); }
private:
  double elapsed(double now, double last) const {
    if (last < 0) { return std::numeric_limits<double>::quiet_NaN(); }
    return (failed() ? failure_time_ : now) - last;
  }
  void fail(double now, const char * reason) {
    failure_time_ = now;
    ready_ = false;
    reason_ = reason;
  }
  void observe(double now, double & last, double timeout, const char * reason) {
    if (failed()) { return; }
    if (last >= 0 && (now < last || now - last > timeout)) {
      stable_since_ = -1;
      if (ready_) {
        fail(now, reason);
        return;
      }
    }
    last = now;
  }
  double start_, warmup_, imu_timeout_, odom_timeout_, startup_timeout_;
  double imu_last_{-1}, odom_last_{-1}, stable_since_{-1};
  double failure_time_{-1};
  bool ready_{false};
  std::string reason_;
};
