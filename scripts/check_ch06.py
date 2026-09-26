#!/usr/bin/env python3
"""Check the default ch06 reference scene: raw IMU semantics, pause, replay, noise."""
import math
import statistics
import rclpy
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, Trigger
from check_ch04 import Probe


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def values(message):
    g, a = message.angular_velocity, message.linear_acceleration
    return (g.x, g.y, g.z, a.x, a.y, a.z)


def main():
    rclpy.init()
    p = Probe("reference")
    samples = []

    def receive(message):
        if stamp(message) == 0:
            samples.clear()
        if samples or stamp(message) == 0:
            samples.append(message)

    p.node.create_subscription(Imu, "/imu/data_raw", receive, qos_profile_sensor_data)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3)  # Let discovery finish before the volatile t=0 sample.
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: len(samples) == 1)
        p.spin(.15)
        assert len(samples) == 1, "Paused wall callbacks must not resample noise"
        first = values(samples[0])
        for _ in range(40):
            assert p.call(p.step, Trigger.Request()).success
        assert [stamp(m) for m in samples] == [i * 5_000_000 for i in range(41)]
        original = [values(m) for m in samples]
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: len(samples) == 1 and stamp(samples[0]) == 0)
        assert values(samples[0]) == first
        for _ in range(40):
            assert p.call(p.step, Trigger.Request()).success
        assert [values(m) for m in samples] == original, "Reset must replay the whole noise stream"

        assert p.call(p.reset, Trigger.Request()).success
        assert p.call(p.pause, SetBool.Request(data=False)).success
        p.wait(lambda: len(samples) >= 2001, timeout=18)
        assert p.call(p.pause, SetBool.Request(data=True)).success
        data = samples[:2001]
        sigmas = (.002, .003, .004, .04, .05, .06)
        truth = (0, 0, .4, 0, .16, 9.81)
        for k, message in enumerate(data):
            assert stamp(message) == k * 5_000_000, "Missing samples; rerun with less host load"
            assert message.header.frame_id == "imu_link"
            assert message.orientation_covariance[0] == -1
            q = message.orientation
            assert (q.x, q.y, q.z, q.w) == (0, 0, 0, 1), "No truth yaw in the raw message"
            for covariance, expected in ((message.angular_velocity_covariance, sigmas[:3]),
                                         (message.linear_acceleration_covariance, sigmas[3:])):
                for i, entry in enumerate(covariance):
                    assert math.isclose(entry, expected[i // 4] ** 2 if i in (0, 4, 8) else 0,
                                        abs_tol=1e-14)
        for axis, (sigma, mean_truth) in enumerate(zip(sigmas, truth)):
            residuals = [values(m)[axis] - mean_truth for m in data]
            mean, measured = statistics.mean(residuals), statistics.pstdev(residuals)
            assert abs(mean) < 5 * sigma / math.sqrt(len(data))
            assert abs(measured - sigma) < .08 * sigma
            print(f"axis={axis} n={len(data)} mean={mean:.8f} stddev={measured:.8f}")
        publishers = p.node.get_publishers_info_by_topic("/imu/data_raw")
        assert len(publishers) == 1
        assert publishers[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert publishers[0].qos_profile.durability == DurabilityPolicy.VOLATILE
        assert p.node.count_publishers("/clock") == 1
        print("PASS raw IMU: units, no orientation, variances, 200 Hz, pause, replay, six-axis noise")
    finally:
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
