#!/usr/bin/env python3
"""Check actual late-subscriber delivery and disk dimensions, with ch01 running."""
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


def main():
    rclpy.init()
    node = rclpy.create_node("check_ch01")
    received = []
    node.create_subscription(
        Marker, "/visualization/robot", received.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    deadline = time.monotonic() + 8.0
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        assert received, "No retained Marker: check launch, ROS_DOMAIN_ID and QoS"
        marker = received[-1]
        assert marker.header.frame_id == "map"
        assert marker.type == Marker.CYLINDER
        assert math.isclose(marker.scale.x, 0.4)
        assert math.isclose(marker.scale.y, 0.4)
        assert marker.color.a == 1.0
        print("PASS: late subscriber received map-frame disk with 0.20 m radius")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
