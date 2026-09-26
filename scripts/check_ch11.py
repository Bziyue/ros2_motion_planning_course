#!/usr/bin/env python3
"""Chapter 11 on actual observed maps: goals, empty failures, isolation and reset."""
import json
from pathlib import Path as FilePath
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import SetBool, Trigger
from check_ch04 import Probe
from check_ch06 import stamp


def main():
    rclpy.init(); p = Probe("ideal"); state = {}
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic, kind, key in [("/map", OccupancyGrid, "map"), ("/planning/grid", OccupancyGrid, "grid"),
                             ("/plan/path", Path, "path"), ("/plan/status", String, "status")]:
        p.node.create_subscription(kind, topic, lambda m, k=key: state.__setitem__(k, m), retained)
    goal_pub = p.node.create_publisher(PoseStamped, "/goal_pose", 1)
    def goal(x, y, frame="map"):
        msg = PoseStamped(); msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y = x, y; msg.pose.orientation.w = 1.
        state.pop("status", None); goal_pub.publish(msg)
        p.wait(lambda: "status" in state)
    def expect(status):
        p.wait(lambda: state.get("status") and state["status"].data == status)
        p.spin(.1)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3)
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: "grid" in state and stamp(state["grid"]) == 0)
        for _ in range(100):
            assert p.call(p.step, Trigger.Request()).success
        p.wait(lambda: stamp(state["grid"]) == 500_000_000)
        goal(-7.5, -8.0); expect("found")
        path = state["path"]
        assert path.header.frame_id == "map" and len(path.poses) >= 2
        assert abs(path.poses[0].pose.position.x+8) < 1e-9
        assert abs(path.poses[-1].pose.position.x+7.5) < 1e-9
        assert all(s.header == path.header and s.pose.orientation.w == 1 for s in path.poses)
        assert p.pose.pose.position.x == -8, "A planned path must not teleport the robot"
        grid, observed = state["grid"], state["map"]
        assert grid.info == observed.info and set(grid.data) == {0, 100}
        assert all(g == 100 for g, raw in zip(grid.data, observed.data) if raw < 0 or raw > 35)
        # Save an actual message snapshot for reproducible teaching plots.
        out = FilePath("tmp/ch11_observed.json"); out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps(dict(resolution=grid.info.resolution, width=grid.info.width,
            height=grid.info.height, origin=[grid.info.origin.position.x, grid.info.origin.position.y],
            observed=list(observed.data), blocked=list(grid.data),
            path=[[s.pose.position.x, s.pose.position.y] for s in path.poses])))
        goal(8.0, 8.0); expect("invalid_goal"); assert not state["path"].poses
        goal(-7.5, -8., "odom"); expect("invalid_goal_frame_or_position"); assert not state["path"].poses
        goal(-7.5, -8.); expect("found")
        topics = {t for t, _ in p.node.get_subscriber_names_and_types_by_node("planner", "/")}
        assert {"/map", "/odometry", "/goal_pose", "/tf", "/tf_static"} <= topics
        assert {i.node_name for i in p.node.get_publishers_info_by_topic("/tf")} == {"truth_odometry"}
        assert not any(t.startswith(("/sim", "/ground_truth", "/visualization")) for t in topics)
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: stamp(state["grid"]) == 0)
        p.spin(.4); assert not state["path"].poses
        assert state["status"].data == "reset"
        print(f"PASS actual observed map: {sum(v == 0 for v in grid.data)} certified free cells, "
              f"{len(path.poses)} path points, no teleport, invalid/unknown goals clear path, TF, isolation, reset")
    finally:
        p.call(p.pause, SetBool.Request(data=True)); p.node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
