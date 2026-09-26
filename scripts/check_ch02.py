#!/usr/bin/env python3
"""Integration check: clock and TF agree, pause freezes time, step adds one dt."""
import math
import time
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.parameter import Parameter


def main():
    rclpy.init()
    node = rclpy.create_node("check_ch02", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    clocks, dynamic, parents = [], [], {}
    node.create_subscription(Clock, "/clock", lambda m: clocks.append(
        m.clock.sec * 10**9 + m.clock.nanosec), 10)

    def collect(msg):
        for tf in msg.transforms:
            child, parent = tf.child_frame_id, tf.header.frame_id
            assert child not in parents or parents[child] == parent, "Multiple TF parents"
            parents[child] = parent
            if child == "base_link":
                dynamic.append(tf)

    node.create_subscription(TFMessage, "/tf", collect, 100)
    node.create_subscription(TFMessage, "/tf_static", collect,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def call(service, kind, request):
        client = node.create_client(kind, service)
        assert client.wait_for_service(timeout_sec=5.0), service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.done() and future.result().success, service
        node.destroy_client(client)

    try:
        spin(1.0)
        call("/sim/pause", SetBool, SetBool.Request(data=True))
        spin(0.2)
        assert clocks and dynamic, "No clock or dynamic TF"
        before = clocks[-1]
        spin(0.2)
        assert clocks[-1] == before, "Paused time advanced"
        call("/sim/step", Trigger, Trigger.Request())
        spin(0.2)
        assert clocks[-1] - before == 5_000_000, "Expected default dt=0.005 s"
        tf = dynamic[-1]
        stamp = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
        assert stamp == clocks[-1]
        assert parents == {"odom": "map", "base_link": "odom",
                           "laser": "base_link", "imu_link": "base_link"}
        t = stamp * 1e-9
        yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
        expected = math.pi / 2.0 + 0.2 * t
        assert abs(math.atan2(math.sin(yaw - expected), math.cos(yaw - expected))) < 1e-10
        call("/sim/reset", Trigger, Trigger.Request())
        spin(1.3)  # Allow the static TF heartbeat to refill reset listener caches.
        assert clocks[-1] == 0
        assert buffer.can_transform("map", "laser", rclpy.time.Time()), "TF missing after reset"
        call("/sim/pause", SetBool, SetBool.Request(data=False))
        spin(0.1)
        assert clocks[-1] > 0
        print("PASS: unique TF parents, analytic yaw, frozen clock, 5 ms step and reset")
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
