#!/usr/bin/env python3
"""Synthetic observed-map fixture: standalone navigation_node, no world or simulator."""
import time
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty, Int64
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from motion2d_interfaces.msg import NavigationReference, NavigationStatus
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data


def main():
    rclpy.init();node=rclpy.create_node('check_navigation_failure');state={};plans=[];stops=[]
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock=node.create_publisher(Clock,'/clock',10);odom=node.create_publisher(Odometry,'/odometry',100)
    scan=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data);grid=node.create_publisher(OccupancyGrid,'/map',retained)
    goal=node.create_publisher(PoseStamped,'/goal_pose',10);ack=node.create_publisher(Int64,'/control/accepted_reference',10)
    node.create_subscription(NavigationReference,'/navigation/reference',lambda m:plans.append(m),10)
    node.create_subscription(NavigationStatus,'/navigation/status',lambda m:state.__setitem__('status',m.state),retained)
    node.create_subscription(Empty,'/navigation/stop',lambda m:stops.append(m),10)
    broadcaster=StaticTransformBroadcaster(node);tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
    def spin(seconds=.08):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def wait(predicate):
        end=time.monotonic()+5
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert predicate(),state
    def at(t,fresh_scan=True):
        ns=round(t*1e9);c=Clock();c.clock.sec=ns//10**9;c.clock.nanosec=ns%10**9;clock.publish(c);spin(.02)
        m=Odometry();m.header.stamp=c.clock;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;odom.publish(m)
        if fresh_scan:
            laser=LaserScan();laser.header.stamp=c.clock;scan.publish(laser)
        spin()
    try:
        wait(lambda:goal.get_subscription_count()==1)
        at(0.)
        m=OccupancyGrid();m.header.frame_id='map';m.info.width=40;m.info.height=40;m.info.resolution=.1;m.info.origin.position.x=-2.;m.info.origin.position.y=-2.;m.info.origin.orientation.w=1.;m.data=[0]*1600;grid.publish(m)
        target=PoseStamped();target.header.frame_id='map';target.pose.position.x=1.;target.pose.orientation.w=1.;goal.publish(target)
        wait(lambda:len(plans)==1);first=plans[-1];start=first.motion.start_time.sec*10**9+first.motion.start_time.nanosec;ack.publish(Int64(data=start));wait(lambda:state.get('status')=='executing')
        spin(.3);assert len(plans)==1,'Paused simulation must not issue more plans'
        # /clock continues and odometry still arrives, but laser stamps stop.
        at(.4,False);wait(lambda:state.get('status')=='stale_observations_braking' and len(stops)==1)
        at(.42);wait(lambda:len(plans)==2)
        # Leave this candidate unacknowledged past its future start.
        at(.5);at(.6);wait(lambda:state.get('status')=='ack_timeout_braking' and len(stops)==2)
        print('PASS navigation failure: immutable free-grid fixture, ACK, paused cadence, live odometry plus stale scan brakes, missing ACK brakes')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
