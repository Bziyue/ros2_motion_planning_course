#!/usr/bin/env python3
"""Actual ch09 ROS pipeline: prediction, corrected scan-time maps, reset and isolation."""
import math
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger
from check_ch04 import Probe
from check_ch06 import stamp
from check_ch08 import pose


def main():
    rclpy.init()
    p = Probe("reference")
    truth, fused, scan_poses, clouds, maps = {}, {}, {}, {}, {}
    for topic, cache in (("/ground_truth/odometry", truth), ("/odometry", fused),
                         ("/odometry/scan", scan_poses)):
        qos = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL) if cache is truth else 1000
        p.node.create_subscription(Odometry, topic, lambda m, c=cache: c.__setitem__(stamp(m), m), qos)
    p.node.create_subscription(PointCloud2, "/cloud/registered", lambda m: clouds.__setitem__(stamp(m), m),
                              qos_profile_sensor_data)
    p.node.create_subscription(OccupancyGrid, "/map", lambda m: maps.__setitem__(stamp(m), m),
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    caches = (truth, fused, scan_poses, clouds, maps)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3)
        for cache in caches:
            cache.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: all(0 in c for c in caches))
        for _ in range(600):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: 3_000_000_000 in fused and 3_000_000_000 in clouds)
        assert sorted(fused) == list(range(0, 3_000_000_001, 5_000_000))
        assert set(scan_poses) == set(clouds) == set(maps)
        assert 25 <= len(scan_poses) <= 31
        x0, y0, yaw0 = pose(truth[0])
        errors = []
        for ns, msg in fused.items():
            tx, ty, tyaw = pose(truth[ns])
            ex, ey, eyaw = pose(msg)
            dx, dy = tx-x0, ty-y0
            errors.append(math.hypot(ex-(math.cos(yaw0)*dx+math.sin(yaw0)*dy),
                                     ey-(-math.sin(yaw0)*dx+math.cos(yaw0)*dy)))
            assert abs(math.remainder(eyaw-tyaw+yaw0, 2*math.pi)) < .04
            assert msg.pose.covariance[0] >= 0 and msg.pose.covariance[35] >= 0
        assert max(errors) < .12, max(errors)
        for name, inputs in (("fusion", {"/imu/data_raw", "/odometry/lidar"}),
                             ("lidar_odometry", {"/scan"})):
            subscriptions = {t for t, _ in p.node.get_subscriber_names_and_types_by_node(name, "/")}
            assert inputs <= subscriptions
            assert not any(t.startswith(("/ground_truth", "/sim", "/tf", "/visualization"))
                           for t in subscriptions), subscriptions
        original = {ns: pose(m) for ns, m in scan_poses.items()}
        p.spin(.3)
        assert len(fused) == 601
        for cache in caches:
            cache.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: all(0 in c for c in caches))
        for _ in range(80):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: 400_000_000 in clouds)
        for ns, msg in scan_poses.items():
            assert pose(msg) == original[ns], "Corrected scan estimates must replay exactly"
        print(f"PASS 601 high-rate predictions, {len(original)} corrected scan/map pairs, "
              f"max aligned position error {max(errors):.6f} m, pause/reset and truth isolation")
    finally:
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
