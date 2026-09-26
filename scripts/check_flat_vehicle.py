#!/usr/bin/env python3
"""Live selected-model acceptance. Truth is read ONLY in this evaluator."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from motion2d_interfaces.msg import AckermannCommand, AckermannTrajectory2D, Trajectory2D


def stamp(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec

def xy(m): return (m.pose.pose.position.x, m.pose.pose.position.y)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',choices=['ackermann','inertial'],default='ackermann')
    p.add_argument('--localization',choices=['truth','slam'],default='truth')
    p.add_argument('--output',default='tmp/flat_vehicle')
    p.add_argument('--expect-failure',action='store_true')
    p.add_argument('--dx',type=float,default=2.)
    p.add_argument('--dy',type=float,default=.3)
    args=p.parse_args();rclpy.init();node=rclpy.create_node('flat_vehicle_evaluator')
    state={};events=[];selected={};truth={};references={};counts={'scan':0,'imu':0,'joint':0};commands=[];curves=[]
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def update(key,m):state[key]=m
    def status(m):state['status']=m.data;events.append(m.data)
    node.create_subscription(String,'/flat/status',status,retained)
    node.create_subscription(String,'/flat/plan_status',lambda m:update('plan',m.data),retained)
    node.create_subscription(String,'/sim/status',lambda m:update('sim_status',m.data),retained)
    node.create_subscription(Odometry,'/odometry',lambda m:(selected.__setitem__(stamp(m),m),update('odom',m)),200)
    node.create_subscription(Odometry,'/ground_truth/odometry',lambda m:truth.__setitem__(stamp(m),m),200)
    node.create_subscription(Odometry,'/flat/reference',lambda m:references.__setitem__(stamp(m),m),200)
    node.create_subscription(OccupancyGrid,'/map',lambda m:update('map',m),retained)
    def sensor(name,m):
        counts[name]+=1;state[name]=m
        if name=='imu':
            assert m.orientation_covariance[0]==-1
            assert all(math.isfinite(x) for x in [m.linear_acceleration.x,m.linear_acceleration.y,m.angular_velocity.z])
    node.create_subscription(Imu,'/imu/data_raw',lambda m:sensor('imu',m),qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',lambda m:sensor('scan',m),qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',lambda m:sensor('joint',m),10)
    if args.model=='ackermann':
        node.create_subscription(AckermannCommand,'/command/ackermann',lambda m:commands.append([m.force,m.steering_rate]),100)
        node.create_subscription(AckermannTrajectory2D,'/flat/ackermann_trajectory',lambda m:(curves.append(m) if m.pieces else None),retained)
    else:
        node.create_subscription(WrenchStamped,'/command/wrench',lambda m:commands.append([m.wrench.force.x,m.wrench.force.y]),100)
        node.create_subscription(Trajectory2D,'/flat/inertial_trajectory',lambda m:(curves.append(m) if m.pieces else None),retained)
    pub=node.create_publisher(PoseStamped,'/goal_pose',1)
    pause=node.create_client(SetBool,'/sim/pause');reset=node.create_client(Trigger,'/sim/reset')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def call(client,request):
        assert client.wait_for_service(timeout_sec=10)
        f=client.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
        assert f.done() and f.result().success
    def wait(predicate,seconds=8):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:spin(.02)
        assert predicate(),f'timeout; state={state.get("status")}, sim={state.get("sim_status")}'
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    try:
        call(reset,Trigger.Request())
        call(pause,SetBool.Request(data=False))
        wait(lambda:'odom' in state and 'map' in state and counts['scan']>=10 and counts['imu']>=100,15)
        start=xy(state['odom']);mission=PoseStamped();mission.header.frame_id='map';mission.pose.orientation.w=1.
        # Both lessons initialize map/odom at the same origin; SLAM map starts at (0,0).
        mission.pose.position.x=start[0]+args.dx;mission.pose.position.y=start[1]+args.dy
        pub.publish(mission)
        deadline=time.monotonic()+60;paused=False
        while time.monotonic()<deadline:
            spin(.04);label=state.get('status','')
            if label=='executing' and not paused:
                call(pause,SetBool.Request(data=True));spin(.1)
                fixed=(stamp(state['odom']),xy(state['odom']));spin(.25)
                assert fixed==(stamp(state['odom']),xy(state['odom'])),'pause changed state/time'
                call(pause,SetBool.Request(data=False));paused=True
            if label=='completed' or label.startswith('plan_failed') or label in ('collision_predicted','map_invalidated_corridor','tracking_margin_exceeded','tracking_did_not_settle'):
                break
        label=state.get('status','timeout');call(pause,SetBool.Request(data=True));spin(.15)
        common=sorted(references.keys() & selected.keys())
        errors=[math.dist(xy(references[k]),xy(selected[k])) for k in common]
        result={'model':args.model,'localization':args.localization,'status':label,'plan':state.get('plan'),
                'sensors':counts,'reference_samples':len(errors),'peak_tracking_error_m':max(errors,default=None),
                'command_axis_peaks':[max((abs(c[i]) for c in commands),default=0) for i in range(2)],
                'pause_checked':paused,'events':events,'sim_status':state.get('sim_status')}
        # Data isolation and unique clock are checked on the actual live graph.
        for name in ['flat_vehicle']+(['lidar_odometry','fusion','slam'] if args.localization=='slam' else []):
            topics=[t for t,_ in node.get_subscriber_names_and_types_by_node(name,'/')]
            assert not any(t.startswith('/ground_truth') or t.startswith('/sim/') or t=='/visualization/world' for t in topics),(name,topics)
        assert node.count_publishers('/clock')==1
        (output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        if args.expect_failure:
            assert label.startswith('plan_failed') and not curves
        else:
            assert label=='completed' and curves and errors and paused
            assert counts['imu']>100 and counts['scan']>10
            assert result['command_axis_peaks'][0]<=2+1e-9
            assert result['command_axis_peaks'][1]<=(1 if args.model=='ackermann' else 2)+1e-9
            if args.model=='ackermann':assert counts['joint']>100
        # Invalid goal clears the retained curve, then reset cannot replay it.
        bad=PoseStamped();bad.header.frame_id='bad';bad.pose.orientation.w=1.;pub.publish(bad)
        wait(lambda:state.get('status')=='invalid_goal')
        call(reset,Trigger.Request());spin(.2)
        call(pause,SetBool.Request(data=False));spin(.8);call(pause,SetBool.Request(data=True));spin(.1)
        assert not state.get('status','').startswith('executing')
        result['invalid_goal_and_reset_passed']=True
        (output/'summary.json').write_text(json.dumps(result,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
