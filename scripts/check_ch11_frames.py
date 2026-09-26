#!/usr/bin/env python3
"""Run against an isolated planner_node: nonidentity TF and bad map boundaries."""
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy

rclpy.init(); node = rclpy.create_node("check_ch11_frames"); state = {}; statuses = []
retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
mp = node.create_publisher(OccupancyGrid, "/map", retained)
op = node.create_publisher(Odometry, "/odometry", 10)
gp = node.create_publisher(PoseStamped, "/goal_pose", 10)
node.create_subscription(Path, "/plan/path", lambda m: state.__setitem__("path", m), retained)
def status(m):
    statuses.append(m.data); state["status"] = m.data
node.create_subscription(String, "/plan/status", status, retained)
def wait(predicate):
    end = time.monotonic()+5
    while not predicate() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.01)
    assert predicate(), state.get("status")
try:
    wait(lambda: mp.get_subscription_count() == op.get_subscription_count() == gp.get_subscription_count() == 1)
    tf = TransformStamped(); tf.header.frame_id = "map"; tf.child_frame_id = "odom"
    tf.transform.translation.x = 1.; tf.transform.translation.y = 2.
    tf.transform.rotation.z = math.sin(math.pi/4); tf.transform.rotation.w = math.cos(math.pi/4)
    broadcaster = StaticTransformBroadcaster(node); broadcaster.sendTransform(tf)
    m = OccupancyGrid(); m.header.frame_id = "map"; m.header.stamp.sec = 1
    m.info.resolution = .1; m.info.width = m.info.height = 80
    m.info.origin.position.x = m.info.origin.position.y = -4.; m.info.origin.orientation.w = 1.
    m.data = [0]*(80*80); mp.publish(m)
    o = Odometry(); o.header.frame_id = "odom"; o.child_frame_id = "base_link"; o.header.stamp.sec = 1
    o.pose.pose.position.x = .4; o.pose.pose.orientation.w = 1.; op.publish(o)
    g = PoseStamped(); g.header.frame_id = "map"; g.pose.position.x = 2.; g.pose.position.y = 2.4
    g.pose.orientation.w = 1.; gp.publish(g)
    wait(lambda: state.get("status") == "found")
    start = state["path"].poses[0].pose.position
    assert abs(start.x-1) < 1e-9 and abs(start.y-2.4) < 1e-9
    m.info.origin.orientation.z = .1; mp.publish(m)
    wait(lambda: state.get("status") == "invalid_map")
    wait(lambda: not state["path"].poses)
    count = len(statuses)
    m.info.origin.orientation.z = 0.; m.data = m.data[:-1]; mp.publish(m)
    wait(lambda: len(statuses) > count and state.get("status") == "invalid_map")
    print("PASS map<-odom rotation+translation, rejected rotated and wrong-sized maps, empty failure path")
finally:
    node.destroy_node(); rclpy.shutdown()
