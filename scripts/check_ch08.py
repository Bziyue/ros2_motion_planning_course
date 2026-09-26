#!/usr/bin/env python3
"""Actual ROS scan-only tracking, gauge alignment, timestamp join and replay."""
import math
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from check_ch04 import Probe
from check_ch06 import stamp


def pose(message):
    p = message.pose.pose
    return p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)


def main():
    rclpy.init()
    p = Probe("reference")
    truth, estimates, clouds, maps = {}, {}, {}, {}
    statuses = []
    p.node.create_subscription(String, "/estimation/status", lambda m: statuses.append(m.data), 100)
    p.node.create_subscription(Odometry, "/ground_truth/odometry",
        lambda m: truth.__setitem__(stamp(m), m),
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    p.node.create_subscription(Odometry, "/odometry",
        lambda m: estimates.__setitem__(stamp(m), m), 100)
    p.node.create_subscription(PointCloud2, "/cloud/registered",
        lambda m: clouds.__setitem__(stamp(m), m), qos_profile_sensor_data)
    p.node.create_subscription(OccupancyGrid, "/map", lambda m: maps.__setitem__(stamp(m), m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3)
        for cache in (truth, estimates, clouds, maps):
            cache.clear()
        statuses.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: all(0 in cache for cache in (truth, estimates, clouds, maps)))
        assert pose(estimates[0]) == (0., 0., 0.)
        for _ in range(600):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: 3_000_000_000 in clouds)
        expected = list(range(0, 3_000_000_001, 100_000_000))
        assert sorted(estimates) == sorted(clouds) == sorted(maps)
        assert set(estimates) <= set(expected) and len(estimates) >= 29
        rejected = len(expected) - len(estimates)
        assert len(statuses) == len(expected)
        assert sum(s not in ("initialized", "converged") for s in statuses) == rejected

        x0, y0, yaw0 = pose(truth[0])
        errors = []
        for ns, estimated in estimates.items():
            tx, ty, tyaw = pose(truth[ns])
            ex, ey, eyaw = pose(estimated)
            dx, dy = tx - x0, ty - y0
            aligned = (math.cos(yaw0)*dx + math.sin(yaw0)*dy,
                       -math.sin(yaw0)*dx + math.cos(yaw0)*dy)
            errors.append(math.hypot(ex-aligned[0], ey-aligned[1]))
            assert abs(math.remainder(eyaw - tyaw + yaw0, 2*math.pi)) < .04
            assert estimated.header.frame_id == "odom" and estimated.child_frame_id == "base_link"
        assert max(errors) < .08, max(errors)
        subscribers = {name for name, _ in p.node.get_subscriber_names_and_types_by_node(
            "lidar_odometry", "/")}
        assert "/scan" in subscribers
        assert not any(t.startswith(("/ground_truth", "/sim", "/tf", "/visualization"))
                       for t in subscribers), subscribers
        assert p.node.count_publishers("/odometry") == 1
        original = {ns: pose(msg) for ns, msg in estimates.items()}
        p.spin(.3)
        assert len(estimates) == len(original), "Pause must not create observations"
        for cache in (truth, estimates, clouds, maps):
            cache.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: 0 in estimates and 0 in clouds)
        for _ in range(80):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: 400_000_000 in clouds)
        for ns, msg in estimates.items():
            assert pose(msg) == original[ns], "Reset must replay estimates as well as sensors"
        print(f"PASS {len(original)} accepted / {rejected} rejected scans: max position error {max(errors):.5f} m, exact timestamps, "
              "map/cloud, no truth inputs, pause/reset replay")
    finally:
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
