#include <cassert>
#include <cmath>
#include "input_gate.hpp"

static InputGate past_startup() {
  InputGate gate(0, 2, .1, .3, 30);
  for (int tick = 0; tick <= 4000; ++tick) {
    const double now = tick * .01;
    gate.imu(now);
    gate.odom(now);
    gate.poll(now);
  }
  assert(gate.poll(40));
  return gate;
}

int main() {
  InputGate g(0, 2, .1, .3, 30);
  // Odometry by itself cannot make the gate ready.
  for (int i=0;i<100;++i) {g.odom(i*.01); assert(!g.poll(i*.01));}
  for (int i=100;i<=310;++i) {
    double t=i*.01; g.imu(t); if(i%10==0)g.odom(t); g.poll(t);
  }
  assert(g.poll(3.10));
  // A late IMU callback must not hide a missed deadline even before poll runs.
  g.imu(3.25); g.odom(3.25); assert(!g.poll(3.25)); assert(g.failed());
  for(int i=326;i<600;++i) {g.imu(i*.01);g.odom(i*.01);assert(!g.poll(i*.01));}
  InputGate empty(0,2,.1,.3,30); assert(!empty.poll(30));assert(empty.failed());
  assert(empty.reason() == "No stable IMU/odometry before startup timeout; check timesync/PTP");
  assert(std::isnan(empty.since_imu(30)));
  InputGate recover(0,1,.1,.3,30);
  recover.imu(0); recover.odom(0); assert(!recover.poll(0));
  assert(!recover.poll(.2)); assert(!recover.failed());
  for(int i=30;i<=140;++i) {double t=i*.01;recover.imu(t);recover.odom(t);recover.poll(t);}
  assert(recover.poll(1.4));
  assert(!recover.poll(1.8)); assert(recover.failed());

  InputGate missed_imu = past_startup();
  missed_imu.odom(40.12);
  assert(!missed_imu.poll(40.12));
  assert(missed_imu.reason() == "IMU interrupted");
  missed_imu.imu(40.125);
  assert(!missed_imu.poll(41));
  assert(std::abs(missed_imu.since_imu(41) - .12) < 1e-9);
  assert(missed_imu.since_odom(41) == 0);

  InputGate missed_odom = past_startup();
  for (int tick = 1; tick <= 7; ++tick) { missed_odom.imu(40 + .05 * tick); }
  assert(!missed_odom.poll(40.35));
  assert(missed_odom.reason() == "Odometry interrupted");
  assert(std::abs(missed_odom.since_odom(41) - .35) < 1e-9);

  InputGate missed_both = past_startup();
  assert(!missed_both.poll(40.35));
  assert(missed_both.reason() == "Input stream stale; restart LIO on the ground");

  InputGate late_callback = past_startup();
  late_callback.imu(40.33);
  late_callback.odom(40.34);
  assert(!late_callback.poll(40.35));
  assert(late_callback.reason() == "IMU interrupted");
  assert(std::abs(late_callback.since_imu(40.35) - .33) < 1e-9);
}
