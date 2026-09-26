#!/usr/bin/env python3
"""Synthetic boundary test: source=trajectory, one /clock, no simulator or truth."""
import math
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from motion2d_interfaces.msg import Trajectory2D,QuinticPiece2D
from rclpy.qos import QoSProfile,DurabilityPolicy

def main():
    rclpy.init();node=rclpy.create_node('check_tracker_boundary');state={};counts={'command':0,'reference':0}
    clock=node.create_publisher(Clock,'/clock',10);odom=node.create_publisher(Odometry,'/odometry',100)
    trajectory=node.create_publisher(Trajectory2D,'/plan/trajectory',1)
    def collect(key,m):state[key]=m;counts[key]=counts.get(key,0)+1
    node.create_subscription(WrenchStamped,'/command/wrench',lambda m:collect('command',m),100)
    node.create_subscription(Odometry,'/control/reference',lambda m:collect('reference',m),100)
    node.create_subscription(String,'/control/status',lambda m:state.__setitem__('status',m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(seconds=.1):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
    def wait(predicate):
        end=time.monotonic()+5
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert predicate(),state.get('status')
    def at(t,v=.0,yaw=.0,observation=True):
        ns=round(t*1e9);c=Clock();c.clock.sec=ns//10**9;c.clock.nanosec=ns%10**9;clock.publish(c);spin(.03)
        m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link';m.header.stamp=c.clock
        m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2);m.twist.twist.linear.x=float(v)
        if observation:odom.publish(m)
        return m
    try:
        wait(lambda:odom.get_subscription_count()==1 and trajectory.get_subscription_count()==1)
        m=at(0.,.2,math.pi/2);wait(lambda:state.get('status')=='waiting_trajectory_braking')
        assert abs(state['command'].wrench.force.x)<1e-12 and abs(state['command'].wrench.force.y+.4)<1e-12
        n=counts['command'];odom.publish(m);spin();assert counts['command']==n
        at(.2,observation=False);wait(lambda:state.get('status')=='stale_odometry_braking')
        assert state['command'].header.stamp.nanosec==200000000
        at(.2);wait(lambda:state.get('status')=='waiting_trajectory_braking')
        plan=Trajectory2D();plan.header.frame_id='odom';plan.header.stamp.nanosec=200000000;plan.start_time.nanosec=300000000
        piece=QuinticPiece2D();piece.duration=2.;piece.x=[0.,0.,0.,.625,-.46875,.09375];piece.y=[0.]*6;plan.pieces=[piece]
        trajectory.publish(plan);wait(lambda:state.get('status')=='trajectory_accepted')
        at(.5);wait(lambda:state.get('status')=='tracking' and 'reference' in state)
        t=.2;C=np.array(piece.x);position=np.polynomial.polynomial.polyval(t,C);velocity=np.polynomial.polynomial.polyval(t,np.polynomial.polynomial.polyder(C));acceleration=np.polynomial.polynomial.polyval(t,np.polynomial.polynomial.polyder(C,2))
        assert abs(state['reference'].pose.pose.position.x-position)<1e-12
        assert abs(state['command'].wrench.force.x-(acceleration+4*position+3*velocity))<1e-12
        at(.7,observation=False);wait(lambda:state.get('status')=='stale_odometry_braking')
        assert state['command'].wrench.force.x==0.,'Do not replay old positive tracking force'
        n=counts['reference'];odom.publish(m);spin()
        assert state['status']=='stale_odometry_braking' and counts['reference']==n,'Queued old data must not restart tracking or reset the epoch'
        at(0.);wait(lambda:state.get('status')=='waiting_trajectory_braking')
        print('PASS tracker boundary: body velocity rotation, duplicate suppression, sim-time watchdog, planned p/v/a feedforward, fresh brake, clock rollback clears trajectory')
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
