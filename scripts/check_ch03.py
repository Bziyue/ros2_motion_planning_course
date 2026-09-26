#!/usr/bin/env python3
"""Verify the default random world's ROS output and shared robot placement."""
import math
import time
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = rclpy.create_node("check_ch03")
    worlds, robots, poses = [], [], []
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MarkerArray, "/visualization/world", worlds.append, retained)
    node.create_subscription(MarkerArray, "/visualization/frames", robots.append, retained)
    node.create_subscription(TFMessage, "/tf",
                             lambda m: poses.extend(t for t in m.transforms
                                                    if t.child_frame_id == "base_link"), 20)
    deadline = time.monotonic() + 8.0
    try:
        while not (worlds and robots and poses) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert worlds and robots and poses, "Missing world, robot or TF"
        markers = worlds[-1].markers
        circles = [m for m in markers if m.ns == "circles"]
        polygons = [m for m in markers if m.ns == "polygons"]
        assert len(circles) == len(polygons) == 8
        assert all(m.type == Marker.CYLINDER and m.scale.x > 0 for m in circles)
        assert all(m.type == Marker.TRIANGLE_LIST and len(m.points) >= 3
                   and len(m.points) % 3 == 0 for m in polygons)
        boundary = next(m for m in markers if m.ns == "boundary")
        assert {(p.x, p.y) for p in boundary.points} == {
            (-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)}
        assert len(robots[-1].markers) == 1, "Chapter 03 should hide the illustrative ray"
        assert math.isclose(robots[-1].markers[0].scale.x, 0.4)
        assert poses[-1].transform.translation.x == -8.0
        assert poses[-1].transform.translation.y == -8.0
        assert node.count_publishers("/clock") == 1
        print("PASS: 8 circles, 8 convex meshes, 20 m boundary, shared start/radius and one clock")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
