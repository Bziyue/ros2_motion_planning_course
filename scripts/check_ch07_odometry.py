#!/usr/bin/env python3
"""Verify the ch07 truth adapter, body twist, acquisition stamps and TF ownership."""
import math
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from check_ch04 import Probe
from check_ch06 import stamp


def main():
    rclpy.init()
    p = Probe("reference")
    p.node.set_parameters([Parameter("use_sim_time", value=True)])
    raw, selected = {}, {}
    p.node.create_subscription(Odometry, "/ground_truth/odometry",
                              lambda m: raw.__setitem__(stamp(m), m),
                              QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    p.node.create_subscription(Odometry, "/odometry",
                              lambda m: selected.__setitem__(stamp(m), m), 100)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        assert p.call(p.reset, Trigger.Request()).success
        p.spin(.2)
        raw.clear()
        selected.clear()
        buffer = Buffer(node=p.node)
        listener = TransformListener(buffer, p.node)
        p.wait(lambda: 0 in raw and 0 in selected)
        for _ in range(40):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: 200_000_000 in selected)
        for ns in range(0, 200_000_001, 5_000_000):
            a, b = raw[ns], selected[ns]
            assert (a.header.frame_id, a.child_frame_id) == ("world", "ground_truth_base")
            assert (b.header.frame_id, b.child_frame_id) == ("odom", "base_link")
            assert a.pose == b.pose and a.twist == b.twist
            t = ns * 1e-9
            assert abs(b.pose.pose.position.x - (-8 + math.sin(.4 * t))) < 1e-10
            assert abs(b.pose.pose.position.y - (-8 + 1 - math.cos(.4 * t))) < 1e-10
            assert abs(b.twist.twist.linear.x - .4) < 1e-10
            assert abs(b.twist.twist.linear.y) < 1e-10
            assert b.twist.twist.angular.z == .4
            assert all(v == 0 for v in b.pose.covariance)
            if ns:
                tf = buffer.lookup_transform("map", "base_link",
                    Time(nanoseconds=ns, clock_type=p.node.get_clock().clock_type))
                assert abs(tf.transform.translation.x - b.pose.pose.position.x) < 1e-10
                assert abs(tf.transform.translation.y - b.pose.pose.position.y) < 1e-10
        publishers = p.node.get_publishers_info_by_topic("/tf")
        assert len(publishers) == 1 and publishers[0].node_name == "truth_odometry"
        assert p.node.count_publishers("/clock") == 1
        assert not buffer.can_transform("world", "ground_truth_base", Time())
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: p.stamp() == 0)
        print("PASS truth odometry: exact stamps, body twist, world/odom alignment, sole TF adapter")
        del listener
    finally:
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
