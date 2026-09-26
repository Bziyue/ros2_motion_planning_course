#!/usr/bin/env python3
"""Observe, certify, explicitly execute; independently evaluate transmitted coefficients."""
import copy
import math
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from motion2d_interfaces.msg import Trajectory2D
from check_ch04 import Probe

def main():
    rclpy.init();p=Probe('trajectory');state={};qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    p.node.create_subscription(Trajectory2D,'/plan/certified_candidate',lambda m:state.__setitem__('candidate',m),qos)
    p.node.create_subscription(String,'/plan/optimization_status',lambda m:state.__setitem__('optimization',m.data),qos)
    p.node.create_subscription(String,'/sim/trajectory_status',lambda m:state.__setitem__('status',m.data),qos)
    p.node.create_subscription(Trajectory2D,'/plan/trajectory',lambda m:state.__setitem__('command',m),1)
    goals=p.node.create_publisher(PoseStamped,'/goal_pose',1)
    execute=p.node.create_client(Trigger,'/trajectory/execute');assert execute.wait_for_service(timeout_sec=5)
    try:
        assert p.call(p.pause,SetBool.Request(data=True)).success
        assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:p.stamp()==0)
        for _ in range(100):assert p.call(p.step,Trigger.Request()).success
        goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=-7.5;goal.pose.position.y=-8.;goal.pose.orientation.w=1.
        goals.publish(goal)
        try:p.wait(lambda:len(state.get('candidate',Trajectory2D()).pieces)>0 and 'continuous_certificate_passed' in state.get('optimization',''))
        except AssertionError:
            print('Candidate diagnostic:',state.get('optimization','no status'),flush=True);raise
        candidate=copy.deepcopy(state['candidate']);assert candidate.header.frame_id=='odom'
        assert candidate.start_time.sec==candidate.start_time.nanosec==0
        assert 'command' not in state and p.pose.pose.position.x==-8
        result=p.call(execute,Trigger.Request());assert result.success,result.message
        p.wait(lambda:'command' in state and state.get('status')=='accepted')
        command=state['command'];assert command.pieces==candidate.pieces
        start=command.start_time.sec+command.start_time.nanosec*1e-9
        duration=sum(piece.duration for piece in command.pieces);errors=np.zeros(3)
        steps=math.ceil((start+duration+.1-p.stamp())/.005)
        for _ in range(steps):
            assert p.call(p.step,Trigger.Request()).success
            t=max(0.,min(duration,p.stamp()-start));piece=command.pieces[-1]
            for item in command.pieces:
                piece=item
                if t<=item.duration+1e-12:break
                t-=item.duration
            t=max(0.,min(t,piece.duration));C=np.array([piece.x,piece.y]);ref=[]
            for d in range(3):
                v=sum((math.factorial(k)/math.factorial(k-d))*C[:,k]*t**(k-d) for k in range(d,6))
                if d>0 and (p.stamp()<start or p.stamp()>start+duration):v=np.zeros(2)
                ref.append(v)
            values=[[p.pose.pose.position.x,p.pose.pose.position.y],[p.velocity.twist.linear.x,p.velocity.twist.linear.y],[p.acceleration.accel.linear.x,p.acceleration.accel.linear.y]]
            errors=np.maximum(errors,np.linalg.norm(np.array(values)-ref,axis=1))
        p.wait(lambda:state['status']=='completed');assert max(errors)<1e-8,errors
        assert abs(p.pose.pose.position.x+7.5)<1e-8
        goal.header.frame_id='bad';goals.publish(goal);p.wait(lambda:not state['candidate'].pieces)
        assert not p.call(execute,Trigger.Request()).success
        assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:p.stamp()==0)
        assert not p.call(execute,Trigger.Request()).success
        print(f'PASS ch16: certified frozen candidate -> explicit command, T={duration:.6f}s, {steps}ticks, p/v/a error={errors.tolist()}; no auto execution, invalid-goal/reset rejection')
    finally:p.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
