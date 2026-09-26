#!/usr/bin/env python3
"""Validate the live observed map, late-join QoS and full reset replay."""
import math
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool, Trigger
from check_ch04 import Probe
from check_ch06 import stamp


def main():
    rclpy.init()
    p = Probe("reference")
    maps = []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    p.node.create_subscription(OccupancyGrid, "/map", maps.append, qos)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3)
        maps.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: maps and stamp(maps[-1]) == 0)
        initial = list(maps[-1].data)
        for _ in range(200):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: stamp(maps[-1]) == 1_000_000_000)
        grid = maps[-1]
        assert grid.header.frame_id == "map"
        assert grid.info.width == grid.info.height == 220
        assert abs(grid.info.resolution - .1) < 1e-8
        assert grid.info.origin.position.x == grid.info.origin.position.y == -11
        assert grid.info.origin.orientation.w == 1
        assert len(grid.data) == 220 * 220 and all(-1 <= v <= 100 for v in grid.data)
        assert any(v >= 65 for v in grid.data) and any(0 <= v <= 35 for v in grid.data)
        counts = [sum(v >= 0 for v in m.data) for m in maps]
        assert counts == sorted(counts), "Observed mask can only grow within a trial"

        def cell(x, y):
            # Metadata resolution is float32; these probes stay away from grid lines.
            ix = math.floor((x + 11) / grid.info.resolution)
            iy = math.floor((y + 11) / grid.info.resolution)
            return grid.data[iy * grid.info.width + ix]

        assert cell(8.25, 8.25) == -1, "Unseen distant goal region must stay unknown"
        assert 0 <= cell(-7.95, -7.95) <= 35
        count = len(maps)
        p.spin(.2)
        assert len(maps) == count
        late = []
        sub = p.node.create_subscription(OccupancyGrid, "/map", late.append, qos)
        p.wait(lambda: late)
        assert late[-1] == grid, "Paused late subscriber must receive retained map"
        p.node.destroy_subscription(sub)
        maps.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: maps and stamp(maps[-1]) == 0)
        assert list(maps[-1].data) == initial, "Reset must remove accumulated evidence"
        print(f"PASS observed map: {counts[0]} -> {counts[-1]} observed cells; unknown, QoS, pause, reset")
    finally:
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
