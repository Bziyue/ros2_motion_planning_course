#!/usr/bin/env python3
"""Run ONLY fusion_node in a separate ROS domain; inject known acquisition times."""
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from check_ch06 import stamp


def main():
    rclpy.init()
    node = rclpy.create_node("check_ch09_time")
    imu_pub = node.create_publisher(Imu, "/imu/data_raw", qos_profile_sensor_data)
    laser_pub = node.create_publisher(Odometry, "/odometry/lidar", 100)
    fused, corrected, statuses = [], [], []
    node.create_subscription(Odometry, "/odometry/estimated", fused.append, 100)
    node.create_subscription(Odometry, "/odometry/scan", corrected.append, 100)
    node.create_subscription(String, "/fusion/status", lambda m: statuses.append(m.data), 100)

    def spin(duration=.08):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)

    def imu(ns):
        m = Imu()
        m.header.frame_id = "imu_link"
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        m.orientation_covariance[0] = -1.
        m.linear_acceleration.x = 2.
        m.linear_acceleration.z = 9.81
        imu_pub.publish(m)
        spin()

    def laser(ns, x):
        m = Odometry()
        m.header.frame_id, m.child_frame_id = "odom", "base_link"
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        m.pose.pose.position.x = x
        m.pose.pose.orientation.w = 1.
        laser_pub.publish(m)
        spin()

    try:
        spin(1.)
        assert imu_pub.get_subscription_count() == laser_pub.get_subscription_count() == 1
        laser(0, 0.)
        assert not fused
        imu(0)
        assert stamp(fused[-1]) == stamp(corrected[-1]) == 0
        imu(7_000_000)
        laser(10_000_000, .0001)  # Future relative to IMU: must wait.
        assert stamp(corrected[-1]) == 0
        imu(14_000_000)
        assert stamp(corrected[-1]) == 10_000_000
        assert math.isclose(corrected[-1].pose.pose.position.x, .0001, abs_tol=1e-12)
        assert math.isclose(fused[-1].pose.pose.position.x, .000196, abs_tol=1e-12)
        imu(21_000_000)
        laser(17_000_000, .000289)  # Delayed, strictly inside the previous IMU interval.
        assert stamp(corrected[-1]) == 17_000_000
        assert math.isclose(corrected[-1].pose.pose.position.x, .000289, abs_tol=1e-12)
        imu(28_000_000)
        assert math.isclose(fused[-1].pose.pose.position.x, .000784, abs_tol=1e-12)
        imu(0); imu(7_000_000); laser(0, 0.)  # Reset IMU first, delayed origin.
        assert stamp(corrected[-1]) == 0
        assert stamp(fused[-1]) == 7_000_000
        assert math.isclose(fused[-1].pose.pose.position.x, .000049, abs_tol=1e-12)
        laser(0, 0.); imu(0)  # Reset laser first, while already near the origin.
        assert stamp(corrected[-1]) == stamp(fused[-1]) == 0
        for k in range(1, 62):
            imu(k * 10_000_000)
        assert "laser_stale" in statuses
        before = len(fused)
        imu(900_000_000)  # Gap above 0.1 s loses initialization; no fabricated new pose.
        assert len(fused) == before
        assert "imu_gap" in statuses
        print("PASS both reset orders, nonaligned/future/delayed laser times, laser stale and IMU gap")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
