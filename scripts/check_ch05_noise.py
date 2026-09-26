#!/usr/bin/env python3
"""Use the empty-room noise lab to measure actual LaserScan residual statistics."""
import math
import statistics
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger


def main():
    rclpy.init()
    node = rclpy.create_node("check_ch05_noise")
    scans = []

    def receive(scan):
        if scan.header.stamp.sec == 0 and scan.header.stamp.nanosec == 0:
            scans.clear()  # The reset frame marks the first sample of this trial.
        if scans or scan.header.stamp.sec == 0 and scan.header.stamp.nanosec == 0:
            scans.append(scan)

    node.create_subscription(LaserScan, "/scan", receive, qos_profile_sensor_data)
    pause = node.create_client(SetBool, "/sim/pause")
    reset = node.create_client(Trigger, "/sim/reset")

    def call(client, request):
        assert client.wait_for_service(timeout_sec=5), "Start the noise lab first"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and future.result().success

    try:
        call(pause, SetBool.Request(data=True))
        call(reset, Trigger.Request())
        call(pause, SetBool.Request(data=False))
        end = time.monotonic() + 12
        while len(scans) < 200 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert len(scans) >= 200, "Run ch05_noise_lab.yaml with model:=ideal"
        residuals = []
        for frame, scan in enumerate(scans[:200]):
            assert len(scan.ranges) == 360 and math.isclose(scan.scan_time, .02, abs_tol=1e-7)
            assert scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec == frame * 20_000_000, \
                "Frames were lost; rerun under lighter system load for exact replay"
            for i, measured in enumerate(scan.ranges):
                # Use the exact configured angle, not rounded float32 metadata.
                angle = -math.pi + i * 2 * math.pi / 360
                expected = 10 / max(abs(math.cos(angle)), abs(math.sin(angle)))
                assert math.isfinite(measured), "The lab is far from range boundaries"
                residuals.append(measured - expected)
        mean, sigma = statistics.mean(residuals), statistics.pstdev(residuals)
        assert abs(mean) < 5 * .01 / math.sqrt(len(residuals)), "Mean exceeds 5 standard errors"
        assert abs(sigma - .01) < .0003, "Measured standard deviation is not 0.01 m"
        print(f"PASS ROS noise: n={len(residuals)}, mean={mean:.8f} m, stddev={sigma:.8f} m; "
              "empty 20 m room, 360 beams, 50 Hz, sigma=.01, seed=4242")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
