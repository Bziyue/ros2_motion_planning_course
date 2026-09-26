#!/usr/bin/env python3
"""Verify off-tick Ackermann IMU and deterministic reset with known held input.
Run only the simulator with the documented zero-noise, 137 Hz IMU/7 Hz laser config.
This evaluator may inspect simulation truth; it never supplies localization.
"""
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger
from motion2d_interfaces.msg import AckermannCommand


def main():
    rclpy.init();node=rclpy.create_node('ackermann_sensor_evaluator');imus={};scans={}
    def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
    node.create_subscription(Imu,'/imu/data_raw',lambda m:imus.__setitem__(stamp(m),m),qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',lambda m:scans.__setitem__(stamp(m),m),qos_profile_sensor_data)
    pub=node.create_publisher(AckermannCommand,'/command/ackermann',1)
    reset=node.create_client(Trigger,'/sim/reset');step=node.create_client(Trigger,'/sim/step')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
    def call(client):
        assert client.wait_for_service(timeout_sec=5)
        f=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=5)
        assert f.done() and f.result().success
    def trial():
        call(reset);spin(.1);imus.clear();scans.clear()
        m=AckermannCommand();m.header.frame_id='base_link';m.force=.8;m.steering_rate=.12;pub.publish(m);spin(.05)
        for _ in range(40):call(step)
        spin(.15)
        assert len(imus)==27, len(imus)  # time-zero message was cleared; floor(0.2*137)
        assert any(ns%5000000 for ns in imus)
        rows=[];peak=0
        for ns,m in sorted(imus.items()):
            t=ns*1e-9;s=.8/.15*(-math.expm1(-.15*t));delta=.12*t
            expected=((.8-.15*s),s*s*math.tan(delta)/.3,s*math.tan(delta)/.3)
            actual=(m.linear_acceleration.x,m.linear_acceleration.y,m.angular_velocity.z)
            peak=max(peak,*(abs(a-b) for a,b in zip(actual,expected)))
            assert m.orientation_covariance[0]==-1 and abs(m.linear_acceleration.z-9.81)<1e-12
            rows.append((ns,*actual))
        assert peak<1e-9,peak
        # The first positive laser acquisition is 1/7 s, not a simulation tick.
        assert 142857143 in scans
        return rows,peak
    try:
        spin(.2);a,peak=trial();b,_=trial();assert a==b
        print(f'27 off-tick IMU samples, 7 Hz laser, exact reset replay; peak analytic error={peak:.3e}')
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
