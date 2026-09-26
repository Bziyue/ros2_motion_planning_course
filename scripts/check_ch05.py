#!/usr/bin/env python3
"""Check ch05 snapshot timing, frame semantics and rendering inputs through ROS."""
import argparse
import math
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from check_ch04 import Probe


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def same_ranges(a, b):
    return len(a) == len(b) and all(
        x == y or (math.isnan(x) and math.isnan(y)) for x, y in zip(a, b))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--beams", type=int, default=720)
    args = parser.parse_args()
    rclpy.init()
    p = Probe("reference")
    p.node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    scans, beams = [], []
    p.node.create_subscription(LaserScan, "/scan", scans.append, qos_profile_sensor_data)
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    p.node.create_subscription(MarkerArray, "/visualization/lidar", beams.append, retained)
    listener = None
    try:
        p.wait(lambda: p.node.count_publishers("/scan") == 1 and len(beams) > 0)
        p.call(p.pause, SetBool.Request(data=True))
        scans.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: scans and stamp(scans[-1]) == 0)
        first = scans[-1]
        assert len(first.ranges) == args.beams and first.header.frame_id == "laser"
        assert len(first.intensities) == 0 and first.time_increment == 0
        assert math.isclose(first.scan_time, .1, abs_tol=1e-7)
        assert math.isclose(first.angle_increment, 2 * math.pi / args.beams, abs_tol=1e-7)
        assert math.isclose(first.angle_max,
                            first.angle_min + (args.beams - 1) * first.angle_increment, abs_tol=1e-6)
        assert first.angle_max < first.angle_min + 2 * math.pi
        assert all(math.isfinite(r) and first.range_min <= r <= first.range_max
                   or math.isnan(r) or math.isinf(r) and r > 0 for r in first.ranges)
        p.spin(.2)
        assert len(scans) == 1, "Pause must not generate duplicate scans"
        # A reset starts a separate trial. Create its TF listener after the reset
        # barrier so queued transforms from the previous trial cannot repopulate it.
        buffer = Buffer(node=p.node)
        listener = TransformListener(buffer, p.node)
        for _ in range(19):
            assert p.call(p.step, Trigger.Request()).success
        assert len(scans) == 1, "No scan before the 20th simulation tick"
        assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: len(scans) == 2)
        assert [stamp(s) for s in scans] == [0, 100_000_000]
        acquired = Time(nanoseconds=100_000_000)
        p.wait(lambda: buffer.can_transform("odom", "laser", acquired))
        tf = buffer.lookup_transform("odom", "laser", acquired).transform
        assert math.isclose(tf.translation.x, p.pose.pose.position.x, abs_tol=1e-10)
        assert math.isclose(tf.translation.y, p.pose.pose.position.y, abs_tol=1e-10)
        p.wait(lambda: stamp(beams[-1].markers[0]) == stamp(scans[-1]))
        hit, miss = beams[-1].markers
        assert hit.header.frame_id == "odom" and miss.header.frame_id == "odom"
        finite = sum(math.isfinite(x) for x in scans[-1].ranges)
        no_return = sum(math.isinf(x) and x > 0 for x in scans[-1].ranges)
        assert len(hit.points) == 2 * finite and len(miss.points) == 2 * no_return
        if hit.points:
            assert math.isclose(hit.points[0].x, tf.translation.x, abs_tol=1e-10)
        # At .105 s the robot has moved, but retained beams stay at their acquisition pose.
        assert p.call(p.step, Trigger.Request()).success
        assert stamp(beams[-1].markers[0]) == 100_000_000
        scans.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: scans and stamp(scans[-1]) == 0)
        assert same_ranges(scans[-1].ranges, first.ranges), "Reset must replay the initial scan"
        info = p.node.get_publishers_info_by_topic("/scan")[0]
        assert info.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert info.qos_profile.durability == DurabilityPolicy.VOLATILE
        assert p.node.count_publishers("/clock") == 1
        print(f"PASS lidar {args.beams}: angles, range encoding, 10 Hz ticks, pause/reset, TF, "
              f"beam origins and SensorDataQoS ({finite} hits, {no_return} no returns at t=.1)")
    finally:
        del listener
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
