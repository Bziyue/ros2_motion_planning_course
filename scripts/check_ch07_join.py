#!/usr/bin/env python3
"""Run with ONLY mapping_node in a separate ROS domain: test both arrival orders."""
import time
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.qos import qos_profile_sensor_data
from check_ch07_cloud import xyz
from check_ch06 import stamp


def main():
    rclpy.init()
    node = rclpy.create_node("check_ch07_join")
    scans = node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    poses = node.create_publisher(Odometry, "/odometry", 100)
    clouds = []
    node.create_subscription(PointCloud2, "/cloud/registered", clouds.append, qos_profile_sensor_data)

    def spin(seconds=.15):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)

    def pose(sec, x):
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = "odom", "base_link"
        msg.header.stamp.sec = sec
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.orientation.w = 1.0
        poses.publish(msg)
        spin()

    def scan(sec, distance=1.0):
        msg = LaserScan()
        msg.header.frame_id, msg.header.stamp.sec = "laser", sec
        msg.angle_increment = .1
        msg.range_min, msg.range_max = .05, 10.0
        msg.ranges = [float(distance), float("inf"), float("nan")]
        scans.publish(msg)
        spin()

    try:
        spin(1)
        assert scans.get_subscription_count() == poses.get_subscription_count() == 1
        pose(1, 3)
        scan(2)  # The latest pose is deliberately one second too old.
        assert not clouds, "Must wait instead of projecting with the latest pose"
        pose(2, 4)
        assert len(clouds) == 1 and xyz(clouds[-1]) == [(5.0, 0.0, 0.0)]
        scan(2)
        assert len(clouds) == 1, "A duplicate scan must not be processed twice"
        pose(3, 6)
        scan(3)
        assert xyz(clouds[-1]) == [(7.0, 0.0, 0.0)]
        scan(0)  # reset, scan first; no old pose at zero is in this trial
        assert len(clouds) == 2
        pose(0, 0)
        assert stamp(clouds[-1]) == 0 and xyz(clouds[-1]) == [(1.0, 0.0, 0.0)]
        pose(1, 3); scan(1)
        pose(0, 0); scan(0)  # reset, odometry first
        assert stamp(clouds[-1]) == 0 and xyz(clouds[-1]) == [(1.0, 0.0, 0.0)]
        count = len(clouds)
        scan(0)  # Repeated reset while already paused at zero must refresh outputs.
        assert len(clouds) == count + 1 and stamp(clouds[-1]) == 0
        scan(1)  # Failed localization: this timestamp will never have a pose.
        pose(2, 4); scan(2)
        assert stamp(clouds[-1]) == 2_000_000_000
        assert xyz(clouds[-1]) == [(5.0, 0.0, 0.0)], "Missing frame must not block newer pairs"
        print("PASS exact-time join: both arrival orders, stale pose rejected, duplicates and paused reset")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
