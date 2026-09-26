#!/usr/bin/env python3
"""Analytic 137 Hz IMU / 7 Hz lidar validation; use ch06_time_lab.yaml."""
import argparse
import math
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from check_ch04 import Probe
from check_ch06 import stamp, values


def grid(index, rate):
    """Integer nearest-nanosecond oracle for integer rates, no floating drift."""
    return (index * 1_000_000_000 + rate // 2) // rate


def analytic_state(t, model):
    """Return x,y,yaw,world ax,world ay,wz; independently evaluate lab dynamics."""
    if model == "reference":
        a = .4 * t
        return (math.sin(a), 1 - math.cos(a), a, -.16 * math.sin(a),
                .16 * math.cos(a), .4)
    t = max(0., t - .1)  # Force starts after 20 paused steps, at t=.1.
    if t == 0:
        return (0.,) * 6
    # m=2, c=.4, F=(1,-.3); Iz=.04, c_w=.02, tau=.01.
    b = -math.expm1(-.2 * t) / .2
    c = (t - b) / .2
    by = -math.expm1(-.5 * t) / .5
    cy = (t - by) / .5
    return (.5 * c, -.15 * c, .25 * cy, .5 * math.exp(-.2 * t),
            -.15 * math.exp(-.2 * t), .25 * by)


def wall_range(x, y, angle):
    """Nearest positive exit of the known 20 m square."""
    dx, dy = math.cos(angle), math.sin(angle)
    tx = ((10 if dx > 0 else -10) - x) / dx if abs(dx) > 1e-12 else math.inf
    ty = ((10 if dy > 0 else -10) - y) / dy if abs(dy) > 1e-12 else math.inf
    return min(tx, ty)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=("reference", "inertial"), default="reference")
    args = parser.parse_args()
    rclpy.init()
    p = Probe(args.model)
    p.node.set_parameters([Parameter("use_sim_time", value=True)])
    imus, scans = [], []

    def receive(collection, message):
        if stamp(message) == 0:
            collection.clear()
        if collection or stamp(message) == 0:
            collection.append(message)

    p.node.create_subscription(Imu, "/imu/data_raw",
                              lambda m: receive(imus, m), qos_profile_sensor_data)
    p.node.create_subscription(LaserScan, "/scan",
                              lambda m: receive(scans, m), qos_profile_sensor_data)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.2)
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: len(imus) == len(scans) == 1)
        p.spin(.2)
        # A fresh consumer after the reset barrier avoids queued previous-trial TF.
        buffer = Buffer(cache_time=Duration(seconds=20), node=p.node)
        listener = TransformListener(buffer, p.node)
        p.spin(.2)
        for tick in range(1, 201):
            if args.model == "inertial" and tick == 21:
                p.command_wrench(1, -.3, .01)
            assert p.call(p.step, Trigger.Request()).success
            now = tick * 5_000_000
            assert stamp(imus[-1]) <= now and stamp(scans[-1]) <= now
            assert grid(len(imus), 137) > now and grid(len(scans), 7) > now
        assert [stamp(m) for m in imus] == [grid(k, 137) for k in range(138)]
        assert [stamp(m) for m in scans] == [grid(k, 7) for k in range(8)]
        for message in imus:
            t = stamp(message) * 1e-9
            x, y, yaw, ax, ay, wz = analytic_state(t, args.model)
            expected = (0, 0, wz, math.cos(yaw) * ax + math.sin(yaw) * ay,
                        -math.sin(yaw) * ax + math.cos(yaw) * ay, 9.81)
            assert max(abs(a - b) for a, b in zip(values(message), expected)) < 1e-10
            assert message.orientation_covariance[0] == -1
            if stamp(message) != 0:  # In tf2, zero means latest, not historical t=0.
                query = Time(nanoseconds=stamp(message), clock_type=p.node.get_clock().clock_type)
                tf = buffer.lookup_transform("odom", "imu_link", query)
                assert abs(tf.transform.translation.x - x) < 1e-10
                assert abs(tf.transform.translation.y - y) < 1e-10
        for message in scans:
            x, y, yaw, *_ = analytic_state(stamp(message) * 1e-9, args.model)
            assert len(message.ranges) == 90
            assert message.time_increment == 0 and math.isclose(message.scan_time, 1 / 7, abs_tol=1e-8)
            for i, measured in enumerate(message.ranges):
                expected = wall_range(x, y, yaw - math.pi + i * 2 * math.pi / 90)
                assert abs(measured - expected) < 2e-6
        counts = len(imus), len(scans)
        p.spin(.2)
        assert counts == (len(imus), len(scans)), "Pause must freeze both sample indices"
        assert p.node.count_publishers("/clock") == p.node.count_publishers("/tf") == 1
        print(f"PASS {args.model}: 138 IMUs, 8 scans; exact acquisition state/TF, no early "
              "samples, 137/7 Hz without tick rounding")

        if args.model == "inertial":
            assert p.call(p.reset, Trigger.Request()).success
            p.command_wrench(2, 0)
            assert p.call(p.pause, SetBool.Request(data=False)).success
            p.wait(lambda: p.status == "collision_predicted", timeout=12)
            p.spin(.1)
            frozen = p.stamp()
            counts = len(imus), len(scans)
            p.spin(.2)
            assert counts == (len(imus), len(scans)) and p.stamp() == frozen
            assert not p.call(p.step, Trigger.Request()).success
            assert stamp(imus[-1]) <= round(frozen * 1e9) < grid(len(imus), 137)
            assert stamp(scans[-1]) <= round(frozen * 1e9) < grid(len(scans), 7)
            print(f"PASS collision: state/time/sensors frozen at t={frozen:.3f} s")
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: len(imus) == len(scans) == 1)
        assert stamp(imus[0]) == stamp(scans[0]) == 0
        print("PASS reset: new sample grids start at zero")
        # Keep the listener alive for the entire trial.
        del listener
    finally:
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
