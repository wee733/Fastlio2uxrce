import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, EstimatorStatusFlags, FailsafeFlags, VehicleAttitude
from rosidl_runtime_py.convert import message_to_ordereddict
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=30);p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init();n=Node('live_lio_validation');records={};subscriptions=[]
def cb(k,m):
    d=message_to_ordereddict(m);d['_recv']=time.time();records.setdefault(k,[]).append(d)
for topic,typ in [('/Odometry',Odometry),('/fmu/out/vehicle_local_position_v1',VehicleLocalPosition),('/fmu/out/vehicle_status_v1',VehicleStatus),('/fmu/out/estimator_status_flags',EstimatorStatusFlags),('/fmu/out/failsafe_flags',FailsafeFlags),('/fmu/out/vehicle_attitude',VehicleAttitude)]:
    subscriptions.append(n.create_subscription(typ,topic,lambda m,k=topic:cb(k,m),20 if topic=='/Odometry' else qos_profile_sensor_data))
end=time.monotonic()+a.seconds
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
out={}
for k,rs in records.items():
    out[k]={'count':len(rs),'last':rs[-1]}
    if k=='/Odometry':
        xyz=np.array([[v['pose']['pose']['position'][c] for c in 'xyz'] for v in rs]);st=np.array([v['header']['stamp']['sec']+v['header']['stamp']['nanosec']*1e-9 for v in rs]);age=np.array([v['_recv'] for v in rs])-st
        out[k].update(rate_hz=float((len(st)-1)/(st[-1]-st[0])),xyz_peak_to_peak_m=np.ptp(xyz,axis=0).tolist(),max_displacement_m=float(np.linalg.norm(xyz-xyz[0],axis=1).max()),max_step_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).max()),age_ms_percentiles=np.percentile(age*1000,[0,50,95,100]).tolist(),max_sample_gap_s=float(np.diff(st).max()),finite=bool(np.isfinite(xyz).all()))
    if 'estimator_status' in k:
        out[k]['flag_true_counts']={f:sum(bool(v[f]) for v in rs) for f in rs[-1] if f.startswith(('cs_ev','cs_tilt','cs_yaw','fs_','reject_'))}
    if 'vehicle_local_position' in k:
        out[k]['valid_counts']={f:sum(bool(v[f]) for v in rs) for f in ['xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control','dead_reckoning']}
    if 'failsafe_flags' in k:out[k]['true_flags']=[f for f,v in rs[-1].items() if v is True]
path=Path(a.output);path.write_text(json.dumps(out,indent=2)+'\n');path.with_suffix('.raw.json').write_text(json.dumps(records)+'\n')
print(json.dumps({k:{f:v for f,v in d.items() if f!='last'} for k,d in out.items()},indent=2));n.destroy_node();rclpy.shutdown()
