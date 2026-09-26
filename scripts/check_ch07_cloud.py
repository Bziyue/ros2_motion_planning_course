#!/usr/bin/env python3
"""Check actual scan -> exact-stamp clouds, reset and consumer data boundaries."""
import math
import struct
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_srvs.srv import SetBool, Trigger
from check_ch04 import Probe
from check_ch06 import stamp


def xyz(cloud):
    """Read the declared XYZ FLOAT32 fields, respecting byte order and point stride."""
    assert cloud.height == 1 and cloud.is_dense
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        ("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)]
    assert cloud.point_step == 12 and cloud.row_step == cloud.width * 12
    assert len(cloud.data) == cloud.row_step
    fmt = ">fff" if cloud.is_bigendian else "<fff"
    return [struct.unpack_from(fmt, cloud.data, i * 12) for i in range(cloud.width)]


def main():
    rclpy.init()
    p = Probe("reference")
    scans, poses, local, registered = {}, {}, {}, {}
    p.node.create_subscription(LaserScan, "/scan", lambda m: scans.__setitem__(stamp(m), m),
                               qos_profile_sensor_data)
    p.node.create_subscription(Odometry, "/odometry", lambda m: poses.__setitem__(stamp(m), m), 100)
    for topic, output in (("/cloud/scan", local), ("/cloud/registered", registered)):
        p.node.create_subscription(PointCloud2, topic,
            lambda m, output=output: output.__setitem__(stamp(m), m), qos_profile_sensor_data)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3)
        for cache in (scans, poses, local, registered):
            cache.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: 0 in registered and 0 in local and 0 in scans and 0 in poses)
        initial = bytes(registered[0].data)
        for _ in range(200):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: 1_000_000_000 in registered)
        assert sorted(scans) == sorted(local) == sorted(registered)
        for ns, scan in scans.items():
            assert local[ns].header.frame_id == "laser"
            assert registered[ns].header.frame_id == "odom"
            points, world = xyz(local[ns]), xyz(registered[ns])
            echoes = [(i, r) for i, r in enumerate(scan.ranges)
                      if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
            assert len(points) == len(world) == len(echoes)
            pose = poses[ns].pose.pose
            yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
            for ((i, distance), a, b) in zip(echoes, points, world):
                angle = scan.angle_min + i * scan.angle_increment
                x, y = distance * math.cos(angle), distance * math.sin(angle)
                assert abs(a[0] - x) < 2e-6 and abs(a[1] - y) < 2e-6 and a[2] == 0
                assert abs(b[0] - (pose.position.x + math.cos(yaw)*x - math.sin(yaw)*y)) < 3e-6
                assert abs(b[1] - (pose.position.y + math.sin(yaw)*x + math.cos(yaw)*y)) < 3e-6
                assert b[2] == 0
        count = len(registered)
        p.spin(.3)
        assert len(registered) == count
        topics = {name for name, _ in p.node.get_subscriber_names_and_types_by_node("mapping", "/")}
        assert "/scan" in topics and "/odometry" in topics
        assert not any(name.startswith(("/ground_truth", "/sim", "/visualization", "/tf"))
                       for name in topics)
        local.clear(); registered.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: 0 in registered)
        assert bytes(registered[0].data) == initial
        print(f"PASS scan clouds: {count} frames, same-stamp SE(2), finite echoes, reset and input isolation")
    finally:
        p.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
