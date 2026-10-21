import argparse
import json
import statistics
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import SensorCombined, TimesyncStatus, VehicleStatus

p=argparse.ArgumentParser()
p.add_argument('--seconds',type=float,default=30)
p.add_argument('--output',required=True)
a=p.parse_args()
rclpy.init()
n=rclpy.create_node('fc_link_timing_probe')
rows=[]
def cb(kind,m):
    d={'kind':kind,'wall_ns':time.time_ns(),'mono_ns':time.monotonic_ns(),'timestamp_us':m.timestamp}
    if kind=='sync':
        d.update(remote_us=m.remote_timestamp,observed_us=m.observed_offset,estimated_us=m.estimated_offset,rtt_us=m.round_trip_time)
    if kind=='imu':
        d.update(gyro_dt=m.gyro_integral_dt,acc_dt=m.accelerometer_integral_dt,relative=m.accelerometer_timestamp_relative)
    if kind=='status':
        d.update(arming_state=m.arming_state)
    rows.append(d)
qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
subs=[n.create_subscription(c,t,lambda m,k=k:cb(k,m),qos) for k,c,t in [('imu',SensorCombined,'/fmu/out/sensor_combined'),('sync',TimesyncStatus,'/fmu/out/timesync_status'),('status',VehicleStatus,'/fmu/out/vehicle_status_v1')]]
end=time.monotonic()+a.seconds
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.1)
Path(a.output).write_text(json.dumps(rows))
for k in ['imu','sync','status']:
    r=[d for d in rows if d['kind']==k]
    if len(r)<2:continue
    age=[(d['wall_ns']/1000-d['timestamp_us'])/1000 for d in r]
    dt=[(y['timestamp_us']-x['timestamp_us'])/1000 for x,y in zip(r,r[1:])]
    recv=[(y['mono_ns']-x['mono_ns'])/1e6 for x,y in zip(r,r[1:])]
    print(k, json.dumps({'n':len(r),'hz':(len(r)-1)*1e9/(r[-1]['mono_ns']-r[0]['mono_ns']),'age_ms':[min(age),statistics.median(age),max(age)],'sample_dt_ms':[min(dt),statistics.median(dt),max(dt)],'receive_dt_ms':[min(recv),statistics.median(recv),max(recv)],'future_over_20ms':sum(v < -20 for v in age),'stale_over_100ms':sum(v > 100 for v in age),'nonincreasing_stamps':sum(v <= 0 for v in dt),'last':r[-1]}))
n.destroy_node()
rclpy.shutdown()
