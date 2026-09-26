#!/usr/bin/env python3
"""Actual ch10 ROS integration: validated loop, rebuilt map, TF ownership and reset."""
import math
import rclpy
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import SetBool, Trigger
from check_ch04 import Probe
from check_ch06 import stamp


def main():
    rclpy.init()
    p = Probe("reference")
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {}; statuses = []; poses = []; alignments = []
    for topic, kind, key in [("/path/graph_before", Path, "before"),
                              ("/path/graph_after", Path, "after"),
                              ("/map", OccupancyGrid, "map"),
                              ("/map/cloud", PointCloud2, "cloud"),
                              ("/slam/edges", MarkerArray, "edges")]:
        p.node.create_subscription(kind, topic, lambda m, k=key: state.__setitem__(k, m), retained)
    p.node.create_subscription(String, "/slam/status", lambda m: statuses.append(m.data), retained)
    p.node.create_subscription(Odometry, "/odometry", poses.append, 1000)
    p.node.create_subscription(TFMessage, "/tf", lambda m: alignments.extend(
        tf for tf in m.transforms if tf.child_frame_id == "odom"), 100)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3); state.clear(); statuses.clear(); poses.clear(); alignments.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: "after" in state and stamp(state["after"]) == 0)
        assert len(state["after"].poses) == 1
        assert p.call(p.pause, SetBool.Request(data=False)).success
        p.wait(lambda: "after" in state and stamp(state["after"]) >= 16_000_000_000, timeout=50)
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.5)
        assert any(s.startswith("loop_accepted") for s in statuses), statuses[-15:]
        before, after = state["before"], state["after"]
        assert before.header.frame_id == after.header.frame_id == "map"
        assert len(before.poses) == len(after.poses) >= 20
        assert before.poses[0].pose == after.poses[0].pose
        differences = [math.hypot(a.pose.position.x-b.pose.position.x,
                                  a.pose.position.y-b.pose.position.y)
                       for a, b in zip(after.poses, before.poses)]
        assert max(differences) > 1e-7
        assert stamp(state["map"]) == stamp(state["cloud"]) == stamp(after)
        assert state["cloud"].header.frame_id == "map" and state["cloud"].width > 100
        assert any(v < 0 for v in state["map"].data) and any(v > 65 for v in state["map"].data)
        loop_marker = next(m for m in state["edges"].markers if m.id == 1)
        assert len(loop_marker.points) >= 2
        assert alignments and all(t.header.frame_id == "map" for t in alignments)
        assert {i.node_name for i in p.node.get_publishers_info_by_topic("/tf")} == {"slam", "estimated_odometry"}
        inputs = {t for t, _ in p.node.get_subscriber_names_and_types_by_node("slam", "/")}
        assert {"/scan", "/odometry/scan"} <= inputs
        assert not any(t.startswith(("/ground_truth", "/sim", "/tf", "/visualization")) for t in inputs)
        jumps = [math.hypot(b.pose.pose.position.x-a.pose.pose.position.x,
                            b.pose.pose.position.y-a.pose.pose.position.y)
                 for a, b in zip(poses, poses[1:]) if stamp(b) > stamp(a)]
        assert max(jumps) < .1, "Graph must not overwrite the local odometry stream"
        frames, loops = len(after.poses), len(loop_marker.points)//2
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: stamp(state["after"]) == 0 and len(state["after"].poses) == 1)
        assert not next(m for m in state["edges"].markers if m.id == 1).points
        print(f"PASS {frames} keyframes, {loops} loops, rebuilt map/cloud, fixed anchor, unique TF, "
              f"no truth inputs, max local odom step {max(jumps):.6f} m and reset")
    finally:
        p.call(p.pause, SetBool.Request(data=True))
        p.node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
