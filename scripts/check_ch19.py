#!/usr/bin/env python3
"""Measure one live ch19.launch.py trial. Truth is read only by this evaluator."""
import argparse
import bisect
import csv
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from motion2d_interfaces.msg import NavigationStatus, MpcStatus
from rclpy.qos import QoSProfile, DurabilityPolicy


def stamp(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec

def pose(m):
    p=m.pose.pose.position; q=m.pose.pose.orientation
    return np.array([p.x,p.y,2*math.atan2(q.z,q.w)])

def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2]); return np.array([a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],math.remainder(a[2]+b[2],2*math.pi)])

def inverse(a):
    c,s=math.cos(a[2]),math.sin(a[2]); return np.array([-c*a[0]-s*a[1],s*a[0]-c*a[1],-a[2]])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--route',choices=['A','B','C'],default='B');parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--seconds',type=float,default=65.);parser.add_argument('--output',default='tmp/ch19_trial');args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('navigation_evaluator');truth={};selected={};references={};alignments={};state={};events=[];qp=[]
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def update(key,m): state[key]=m
    node.create_subscription(Odometry,'/ground_truth/odometry',lambda m:truth.__setitem__(stamp(m),pose(m)),200)
    node.create_subscription(Odometry,'/odometry',lambda m:selected.__setitem__(stamp(m),pose(m)),200)
    node.create_subscription(Odometry,'/control/reference',lambda m:references.__setitem__(stamp(m),pose(m)),200)
    node.create_subscription(PoseStamped,'/sim/pose',lambda m:update('sim',m),retained)
    node.create_subscription(String,'/sim/status',lambda m:update('sim_status',m.data),retained)
    node.create_subscription(OccupancyGrid,'/map',lambda m:update('map',m),retained)
    def navigation(m):
        state['navigation']=m
        events.append([stamp(m)*1e-9,m.state,m.planner_status,m.optimization_status,m.planning_seconds,m.accepted_plans,m.stop_count])
    node.create_subscription(NavigationStatus,'/navigation/status',navigation,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(MpcStatus,'/control/mpc_status',lambda m:qp.append([stamp(m)*1e-9,m.status,m.compute_seconds,m.max_violation]),100)
    def transform(m):
        for tf in m.transforms:
            if tf.header.frame_id=='map' and tf.child_frame_id=='odom':
                p=tf.transform.translation;q=tf.transform.rotation;alignments[stamp(tf)]=np.array([p.x,p.y,2*math.atan2(q.z,q.w)])
    node.create_subscription(TFMessage,'/tf',transform,100)
    goal=node.create_publisher(PoseStamped,'/goal_pose',10);pause=node.create_client(SetBool,'/sim/pause')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def call(value):
        future=pause.call_async(SetBool.Request(data=value));rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        assert future.done() and future.result().success
    def simtime():return stamp(state['sim'])*1e-9 if 'sim' in state else 0.
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    try:
        assert pause.wait_for_service(timeout_sec=10);call(False)
        deadline=time.monotonic()+12
        while (not selected or 'map' not in state or goal.get_subscription_count()<1) and time.monotonic()<deadline:spin(.05)
        assert selected and 'map' in state,'No localization/map during startup'
        initial_map=np.array(state['map'].data);unknown_initial=float(np.mean(initial_map<0))
        mission=PoseStamped();mission.header.frame_id='map';mission.pose.position.x=4. if args.route=='C' else -4.;mission.pose.position.y=mission.pose.position.x;mission.pose.orientation.w=1.
        mission.header.stamp=state['sim'].header.stamp;start=simtime();goal.publish(mission)
        deadline=time.monotonic()+args.seconds*1.8+10;next_print=start+5;reason='time_limit'
        while simtime()-start<args.seconds and time.monotonic()<deadline:
            spin(.05)
            nav=state.get('navigation');label=nav.state if nav else 'waiting'
            if simtime()>=next_print:
                print(f'{args.route}/{args.seed} t={simtime():.1f} {label} {nav.planner_status if nav else ""} plans={nav.accepted_plans if nav else 0}',flush=True);next_print+=5
            if state.get('sim_status')=='collision_predicted':reason='collision_predicted';break
            if label=='reached':reason='reached';break
        call(True);spin(.2)
        nav=state.get('navigation');keys=sorted(selected.keys() & truth.keys());assert len(keys)>20
        tfkeys=sorted(alignments)
        def mapped(ns):
            index=bisect.bisect_right(tfkeys,ns)-1
            return compose(alignments[tfkeys[index]],selected[ns]) if index>=0 else selected[ns]
        # A single initial SE(2) alignment, never fit the whole path to hide drift.
        anchor=compose(truth[keys[0]],inverse(mapped(keys[0])))
        rows=[];errors=[];tracking=[];rpe=[]
        for ns in keys:
            actual=truth[ns];estimated=compose(anchor,mapped(ns));error=np.linalg.norm(estimated[:2]-actual[:2]);errors.append(error)
            tracking_error=np.linalg.norm(selected[ns][:2]-references[ns][:2]) if ns in references else math.nan
            if math.isfinite(tracking_error):tracking.append(tracking_error)
            if ns+10**9 in truth and ns+10**9 in selected:
                truth_delta=compose(inverse(actual),truth[ns+10**9]);estimated_delta=compose(inverse(selected[ns]),selected[ns+10**9]);rpe.append(np.linalg.norm(compose(inverse(truth_delta),estimated_delta)[:2]))
            rows.append([ns*1e-9,*actual,*estimated,tracking_error])
        def write(name,header,values):
            with (out/name).open('w') as f:w=csv.writer(f);w.writerow(header);w.writerows(values)
        write('samples.csv',['t','truth_x','truth_y','truth_yaw','estimated_x','estimated_y','estimated_yaw','tracking_error'],rows)
        write('events.csv',['t','state','planner','optimizer','planning_seconds','accepted','stops'],events)
        write('mpc.csv',['t','status','seconds','max_violation'],qp)
        solves=[e[4] for e in events if e[1] in ('awaiting_ack','planning_failed_braking') and e[0]>=start]
        position=truth[keys[-1]][:2];true_goal_error=float(np.linalg.norm(position-[-4.,-4.]))
        collision=reason=='collision_predicted'
        result={'route':args.route,'seed':args.seed,'reason':reason,'success':reason=='reached' and true_goal_error<.25,'sim_seconds':simtime()-start,
                'collision_attempts':int(collision),'accepted_plans':int(nav.accepted_plans) if nav else 0,'stops':int(nav.stop_count) if nav else 0,
                'ATE_initial_aligned_map_RMSE_m':float(np.sqrt(np.mean(np.square(errors)))),'RPE_1s_translation_RMSE_m':float(np.sqrt(np.mean(np.square(rpe)))) if rpe else None,
                'tracking_RMSE_m':float(np.sqrt(np.mean(np.square(tracking)))) if tracking else None,'true_goal_error_m':true_goal_error,
                'planning_p95_ms':float(np.quantile(solves,.95)*1000) if solves else None,'mpc_p95_ms':float(np.quantile([q[2] for q in qp],.95)*1000) if qp else None,
                'mpc_failures':sum(q[1]!='solved' for q in qp),'unknown_initial_fraction':unknown_initial,'unknown_final_fraction':float(np.mean(np.array(state['map'].data)<0)),
                'localization_samples':len(keys),'reference_samples':len(tracking)}
        # Assert graph isolation, without judging a failed mission as a successful one.
        for name in ['navigation','tracker']+(['lidar_odometry','fusion','slam'] if args.route=='C' else []):
            topics=[t for t,_ in node.get_subscriber_names_and_types_by_node(name,'/')]
            assert not any(t.startswith('/ground_truth') or t.startswith('/sim/') or t=='/visualization/world' for t in topics),(name,topics)
        assert node.count_publishers('/clock')==1
        (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
