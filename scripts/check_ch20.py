#!/usr/bin/env python3
"""Capture a paired CPU/CUDA replay and publication-phase timings from one live launch."""
import argparse
import csv
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from motion2d_interfaces.msg import LidarTiming
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import qos_profile_sensor_data
from check_ch04 import Probe
from check_ch05 import stamp


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--backend',choices=['cpu','cuda'],required=True);parser.add_argument('--beams',type=int,default=720);parser.add_argument('--output',required=True);args=parser.parse_args()
    rclpy.init();p=Probe('reference');scans={};timings={}
    p.node.create_subscription(LaserScan,'/scan',lambda m:scans.__setitem__(stamp(m),m),qos_profile_sensor_data)
    p.node.create_subscription(LidarTiming,'/sim/lidar_timing',lambda m:timings.__setitem__(stamp(m),m),100)
    try:
        p.spin(.2);p.call(p.pause,SetBool.Request(data=True));scans.clear();timings.clear()
        assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:0 in scans and 0 in timings)
        first=np.array(scans[0].ranges);assert first.size==args.beams
        p.call(p.pause,SetBool.Request(data=False));deadline=time.monotonic()+25
        while p.stamp()<10 and p.status!='collision_predicted' and time.monotonic()<deadline:p.spin(.03)
        p.call(p.pause,SetBool.Request(data=True));p.spin(.2);assert p.stamp()>=10,p.status
        keys=list(range(0,10000000001,100000000));assert all(k in scans and k in timings for k in keys),'Dropped scan or timing'
        rows=[];fields=['scan_seconds','noise_seconds','message_seconds','publish_seconds','total_seconds','kernel_seconds','download_seconds']
        for ns in keys:
            scan=scans[ns];timing=timings[ns]
            assert scan.header.frame_id=='laser' and scan.time_increment==0 and len(scan.ranges)==args.beams
            assert timing.backend==args.backend and timing.beams==args.beams
            assert abs(sum(getattr(timing,k) for k in fields[:4])-timing.total_seconds)<1e-9
            if args.backend=='cpu':assert np.isnan(timing.kernel_seconds) and np.isnan(timing.download_seconds)
            else:assert timing.kernel_seconds>=0 and timing.download_seconds>=0
            rows.append([ns*1e-9]+[getattr(timing,k) for k in fields])
        out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
        values=np.array([list(scans[k].ranges) for k in keys],dtype=np.float32);np.savez(out/'scans.npz',stamps=np.array(keys),ranges=values)
        with (out/'timings.csv').open('w') as f:w=csv.writer(f);w.writerow(['t']+fields);w.writerows(rows)
        warm=np.array(rows)[10:];result={'backend':args.backend,'beams':args.beams,'scans':len(keys),'measured_after_warmup':len(warm)}
        for j,name in enumerate(fields,1):
            for fraction in [.5,.95]:result[name.removesuffix('_seconds')+f'_p{int(fraction*100)}_ms']=float(np.quantile(warm[:,j],fraction)*1000) if np.isfinite(warm[:,j]).all() else None
        (out/'summary.json').write_text(json.dumps(result,indent=2))
        scans.clear();assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:0 in scans)
        np.testing.assert_array_equal(first,np.array(scans[0].ranges));assert p.node.count_publishers('/clock')==1
        print(json.dumps(result),flush=True)
    finally:p.node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
