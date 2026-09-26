#!/usr/bin/env python3
"""Exercise chapter 04 through ROS, including tick boundaries and failed motion."""
import math
import time
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class Probe:
    """Small synchronous experiment driver; the simulator still owns /clock."""
    def __init__(self):
        self.node = rclpy.create_node("check_ch04")
        self.pose = None
        self.velocity = None
        self.status = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(PoseStamped, "/sim/pose",
                                      lambda m: setattr(self, "pose", m), qos)
        self.node.create_subscription(TwistStamped, "/sim/velocity",
                                      lambda m: setattr(self, "velocity", m), qos)
        self.node.create_subscription(String, "/sim/status",
                                      lambda m: setattr(self, "status", m.data), qos)
        self.pose_pub = self.node.create_publisher(PoseStamped, "/command/pose", 1)
        self.pause = self.node.create_client(SetBool, "/sim/pause")
        self.step = self.node.create_client(Trigger, "/sim/step")
        self.reset = self.node.create_client(Trigger, "/sim/reset")
        for client in (self.pause, self.step, self.reset):
            assert client.wait_for_service(timeout_sec=5), "Start ch04.launch.py first"
        self.wait(lambda: self.pose is not None and self.pose_pub.get_subscription_count() == 1)

    def wait(self, predicate, timeout=5):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        assert predicate(), "Timed out waiting for simulator output"

    def spin(self, duration=0.08):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)
        assert future.done(), "Service timed out"
        self.spin(0.02)
        return future.result()

    def command_pose(self, x, y, frame="odom"):
        message = PoseStamped()
        message.header.frame_id = frame
        message.pose.position.x, message.pose.position.y = float(x), float(y)
        message.pose.orientation.w = 1.0
        self.pose_pub.publish(message)
        self.spin()

    def stamp(self):
        return self.pose.header.stamp.sec + self.pose.header.stamp.nanosec * 1e-9


def check_ideal(probe):
    assert probe.call(probe.reset, Trigger.Request()).success
    probe.wait(lambda: probe.stamp() == 0.0)
    assert probe.pose.pose.position.x == -8.0
    probe.command_pose(-7, -8)
    assert probe.stamp() == 0.0 and probe.pose.pose.position.x == -8.0
    assert probe.call(probe.step, Trigger.Request()).success
    probe.wait(lambda: math.isclose(probe.stamp(), 0.005))
    assert probe.pose.pose.position.x == -7.0
    assert probe.velocity.twist.linear.x == 0.0
    probe.command_pose(-6, -8, frame="wrong_frame")
    assert probe.call(probe.step, Trigger.Request()).success
    assert probe.pose.pose.position.x == -7.0
    before = probe.stamp()
    probe.command_pose(20, -8)
    assert not probe.call(probe.step, Trigger.Request()).success
    probe.wait(lambda: probe.status == "collision_predicted")
    assert probe.stamp() == before and probe.pose.pose.position.x == -7.0
    assert not probe.call(probe.pause, SetBool.Request(data=False)).success
    assert probe.call(probe.reset, Trigger.Request()).success
    assert probe.call(probe.step, Trigger.Request()).success
    probe.wait(lambda: math.isclose(probe.stamp(), 0.005))
    assert probe.pose.pose.position.x == -8.0, "Reset must clear pending commands"
    assert probe.node.count_publishers("/clock") == 1
    print("PASS ideal: paused command, one tick, invalid frame, blocked sweep, reset and one clock")


def main():
    rclpy.init()
    probe = Probe()
    try:
        check_ideal(probe)
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
