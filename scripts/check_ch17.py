#!/usr/bin/env python3
"""Real ROS closed-loop measurements, aligned by observation/reference timestamps."""
import csv
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from check_ch04 import Probe
from check_ch06 import stamp

def main(output="tmp/ch17_ros",mpc=False):
    rclpy.init();p=Probe('inertial');truth={};references={};commands={};status={};diagnostics={};predictions={}
    p.node.create_subscription(Odometry,'/ground_truth/odometry',lambda m:truth.__setitem__(stamp(m),m),100)
    p.node.create_subscription(Odometry,'/control/reference',lambda m:references.__setitem__(stamp(m),m),100)
    p.node.create_subscription(WrenchStamped,'/command/wrench',lambda m:commands.__setitem__(stamp(m),m),100)
    p.node.create_subscription(String,'/control/status',lambda m:status.__setitem__('value',m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if mpc:
        from motion2d_interfaces.msg import MpcStatus
        from nav_msgs.msg import Path as RosPath
        p.node.create_subscription(MpcStatus,'/control/mpc_status',lambda m:diagnostics.__setitem__(stamp(m),m),100)
        p.node.create_subscription(RosPath,'/control/prediction',lambda m:predictions.__setitem__(stamp(m),m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    enable=p.node.create_client(SetBool,'/tracker/enable');assert enable.wait_for_service(timeout_sec=5)
    try:
        assert p.call(p.pause,SetBool.Request(data=True)).success
        # Advance before reset to exercise an actual backwards jump, including tracker epoch.
        assert p.call(p.step,Trigger.Request()).success
        assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:p.stamp()==0)
        assert p.call(enable,SetBool.Request(data=True)).success
        p.spin(.1);truth.clear();references.clear();commands.clear();diagnostics.clear();predictions.clear()
        assert p.call(p.pause,SetBool.Request(data=False)).success
        deadline=time.monotonic()+30
        while p.stamp()<12 and time.monotonic()<deadline:rclpy.spin_once(p.node,timeout_sec=.01)
        assert p.stamp()>=12,'Simulation failed to advance'
        assert p.call(p.pause,SetBool.Request(data=True)).success;p.spin(.15)
        rows=[]
        for ns in sorted(references.keys() & truth.keys()):
            actual=truth[ns];ref=references[ns];a=actual.pose.pose.position;b=ref.pose.pose.position
            yaw=lambda q:2*math.atan2(q.z,q.w)
            ey=math.remainder(yaw(ref.pose.pose.orientation)-yaw(actual.pose.pose.orientation),2*math.pi)
            rows.append([ns*1e-9,b.x,b.y,a.x,a.y,math.hypot(b.x-a.x,b.y-a.y),ey,4-max(abs(a.x),abs(a.y))-.2])
        assert len(rows)>500,len(rows)
        values=np.array(rows);rms=float(np.sqrt(np.mean(values[:,5]**2)));maximum=float(values[:,5].max())
        assert rms<.03 and maximum<.08,(rms,maximum)
        assert values[:,7].min()>2.5
        assert commands and max(max(abs(m.wrench.force.x),abs(m.wrench.force.y)) for m in commands.values())<=2+1e-9
        stamps=sorted(references);gaps=np.diff(stamps)*1e-9
        assert gaps.min()>=.019999, gaps.min()
        count=len(references);p.spin(.2);assert len(references)==count
        topics=[t for t,_ in p.node.get_subscriber_names_and_types_by_node('tracker','/')]
        assert not any('ground_truth' in t or t.startswith('/sim/') or t.startswith('/tf') for t in topics),topics
        force_peak=max(max(abs(m.wrench.force.x),abs(m.wrench.force.y)) for m in commands.values())
        measured=[diagnostics[ns] for ns in stamps if ns in diagnostics]
        paths=[predictions[ns] for ns in stamps if ns in predictions and predictions[ns].poses]
        assert p.call(enable,SetBool.Request(data=False)).success;p.wait(lambda:status.get('value')=='disabled_braking')
        references.clear()
        assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:p.stamp()==0)
        assert p.call(enable,SetBool.Request(data=True)).success
        for _ in range(4):assert p.call(p.step,Trigger.Request()).success
        p.wait(lambda:any(ns<=20000000 for ns in references))
        Path(output).mkdir(parents=True,exist_ok=True)
        with open(Path(output)/'samples.csv','w') as f:
            w=csv.writer(f);w.writerow(['t','x_ref','y_ref','x','y','error','yaw_error','clearance']);w.writerows(rows)
        result={'matched_samples':len(rows),'rms_m':rms,'max_m':maximum,'yaw_rms_rad':float(np.sqrt(np.mean(values[:,6]**2))),
                'min_clearance_m':float(values[:,7].min()),'force_peak_N':force_peak}
        if mpc:
            assert len(measured)>.95*len(stamps)
            assert all(m.status=='solved' for m in measured),{m.status for m in measured}
            assert max(m.max_violation for m in measured)<1e-4
            assert len(paths)>.9*len(stamps)
            for path in paths:
                assert path.header.frame_id=='odom' and len(path.poses)==20
                assert abs(stamp(path.poses[0])-stamp(path)-20000000)<2
            result.update(solved=len(measured),compute_p95_ms=float(np.quantile([m.compute_seconds for m in measured],.95)*1000))
        (Path(output)/'summary.json').write_text(json.dumps(result,indent=2))
        print('PASS ch18 MPC ROS:' if mpc else 'PASS ch17 ROS:',json.dumps(result),'; body/odom tracking, cadence, paused deduplication, reset, disable/brake, input isolation')
    finally:
        p.call(p.pause,SetBool.Request(data=True));p.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
