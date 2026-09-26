#!/usr/bin/env python3
"""Exercise chapter 04 through ROS, including tick boundaries and failed motion."""
import argparse
import math
import time
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped, AccelStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class Probe:
    """Small synchronous experiment driver; the simulator still owns /clock."""
    def __init__(self, model="ideal"):
        self.node = rclpy.create_node("check_ch04")
        self.pose = None
        self.velocity = None
        self.status = None
        self.acceleration = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(PoseStamped, "/sim/pose",
                                      lambda m: setattr(self, "pose", m), qos)
        self.node.create_subscription(TwistStamped, "/sim/velocity",
                                      lambda m: setattr(self, "velocity", m), qos)
        self.node.create_subscription(String, "/sim/status",
                                      lambda m: setattr(self, "status", m.data), qos)
        self.node.create_subscription(AccelStamped, "/sim/acceleration",
                                      lambda m: setattr(self, "acceleration", m), qos)
        self.pose_pub = self.node.create_publisher(PoseStamped, "/command/pose", 1)
        self.velocity_pub = self.node.create_publisher(TwistStamped, "/command/velocity", 1)
        self.wrench_pub = self.node.create_publisher(WrenchStamped, "/command/wrench", 1)
        self.pause = self.node.create_client(SetBool, "/sim/pause")
        self.step = self.node.create_client(Trigger, "/sim/step")
        self.reset = self.node.create_client(Trigger, "/sim/reset")
        for client in (self.pause, self.step, self.reset):
            assert client.wait_for_service(timeout_sec=5), "Start ch04.launch.py first"
        active_pub = {"ideal": self.pose_pub, "velocity": self.velocity_pub,
                      "inertial": self.wrench_pub, "reference": None, "trajectory": None}[model]
        self.wait(lambda: self.pose is not None and
                  (active_pub is None or active_pub.get_subscription_count() == 1))

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

    def command_velocity(self, vx, vy, omega=0.0, frame="odom"):
        message = TwistStamped()
        message.header.frame_id = frame
        message.twist.linear.x, message.twist.linear.y = float(vx), float(vy)
        message.twist.angular.z = float(omega)
        self.velocity_pub.publish(message)
        self.spin()

    def command_wrench(self, fx, fy, torque=0.0, frame="odom"):
        message = WrenchStamped()
        message.header.frame_id = frame
        message.wrench.force.x, message.wrench.force.y = float(fx), float(fy)
        message.wrench.torque.z = float(torque)
        self.wrench_pub.publish(message)
        self.spin()


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
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=("ideal", "velocity", "inertial", "reference"), default="ideal")
    parser.add_argument("--mass", type=float, default=1.0,
                        help="Expected inertial mass; assumes radius=.2 and automatic inertia")
    args = parser.parse_args()
    rclpy.init()
    probe = Probe(args.model)
    try:
        if args.model == "reference":
            check_reference(probe)
        elif args.model == "inertial":
            check_inertial(probe, args.mass)
        else:
            (check_ideal if args.model == "ideal" else check_velocity)(probe)
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()


def check_velocity(p):
    assert p.call(p.reset, Trigger.Request()).success
    p.wait(lambda: p.stamp() == 0.0)
    assert p.pose_pub.get_subscription_count() == 0, "Inactive pose input must not be subscribed"
    p.command_velocity(10, 0, 2)
    assert p.stamp() == 0.0 and p.pose.pose.position.x == -8.0
    for _ in range(10):
        assert p.call(p.step, Trigger.Request()).success
    p.wait(lambda: math.isclose(p.stamp(), .05))
    assert math.isclose(p.pose.pose.position.x, -7.95, abs_tol=1e-10)
    assert p.velocity.twist.linear.x == 1.0 and p.velocity.twist.angular.z == 1.0
    x0, y0 = p.pose.pose.position.x, p.pose.pose.position.y
    p.command_velocity(.5, 0, frame="base_link")
    assert p.call(p.step, Trigger.Request()).success
    assert math.isclose(p.pose.pose.position.x - x0, .0025 * math.cos(.05), abs_tol=1e-10)
    assert math.isclose(p.pose.pose.position.y - y0, .0025 * math.sin(.05), abs_tol=1e-10)
    for _ in range(100):
        assert p.call(p.step, Trigger.Request()).success
    p.wait(lambda: p.status == "command_timeout")
    stopped = (p.pose.pose.position.x, p.pose.pose.position.y)
    for _ in range(5):
        assert p.call(p.step, Trigger.Request()).success
    assert stopped == (p.pose.pose.position.x, p.pose.pose.position.y)
    assert p.velocity.twist.linear.x == p.velocity.twist.linear.y == 0.0
    assert p.call(p.reset, Trigger.Request()).success
    assert p.call(p.step, Trigger.Request()).success
    assert p.pose.pose.position.x == -8.0
    print("PASS velocity: input isolation, pause, norm/yaw limits, body frame, timeout and reset")


def check_inertial(p, mass):
    assert p.call(p.reset, Trigger.Request()).success
    p.wait(lambda: p.stamp() == 0.0)
    assert p.pose_pub.get_subscription_count() == p.velocity_pub.get_subscription_count() == 0
    p.command_wrench(1, 0, frame="base_link")
    assert p.call(p.step, Trigger.Request()).success
    assert p.pose.pose.position.x == -8.0
    p.call(p.reset, Trigger.Request())
    p.command_wrench(99, 0, 99)
    assert p.pose.pose.position.x == -8.0, "Paused force must not advance motion"
    for _ in range(10):
        assert p.call(p.step, Trigger.Request()).success
    assert math.isclose(p.pose.pose.position.x, -8 + .0025 / mass, abs_tol=1e-10)
    assert math.isclose(p.velocity.twist.linear.x, .1 / mass, abs_tol=1e-10)
    assert math.isclose(p.velocity.twist.angular.z, .5 / mass, abs_tol=1e-10)
    p.call(p.reset, Trigger.Request())
    p.command_wrench(1, 0)
    for _ in range(100):
        assert p.call(p.step, Trigger.Request()).success
    for _ in range(5):  # Input expires; zero drag means coasting, not instant stopping.
        assert p.call(p.step, Trigger.Request()).success
    p.wait(lambda: p.status == "command_timeout")
    assert math.isclose(p.velocity.twist.linear.x, .5 / mass, abs_tol=1e-10)
    assert math.isclose(p.pose.pose.position.x, -8 + .1375 / mass, abs_tol=1e-10)
    assert p.acceleration.accel.linear.x == 0.0
    p.command_wrench(-1, 0)
    for _ in range(100):
        assert p.call(p.step, Trigger.Request()).success
    assert abs(p.velocity.twist.linear.x) < 1e-10
    assert math.isclose(p.pose.pose.position.x, -8 + .2625 / mass, abs_tol=1e-10)
    p.call(p.reset, Trigger.Request())
    assert p.call(p.step, Trigger.Request()).success
    assert p.pose.pose.position.x == -8.0 and p.velocity.twist.linear.x == 0.0
    print(f"PASS inertial mass={mass}: input isolation, limits, torque, coast, braking and reset")


def check_reference(p):
    assert p.call(p.reset, Trigger.Request()).success
    p.wait(lambda: p.stamp() == 0.0)
    assert p.pose_pub.get_subscription_count() == p.velocity_pub.get_subscription_count() == 0
    assert p.wrench_pub.get_subscription_count() == 0
    assert math.isclose(p.velocity.twist.linear.x, .4)
    assert math.isclose(p.acceleration.accel.linear.y, .16)
    p.spin(.2)
    assert p.stamp() == 0.0
    for _ in range(100):
        assert p.call(p.step, Trigger.Request()).success
    assert math.isclose(p.stamp(), .5)
    assert math.isclose(p.pose.pose.position.x, -8 + math.sin(.2), abs_tol=1e-10)
    assert math.isclose(p.pose.pose.position.y, -8 + 1 - math.cos(.2), abs_tol=1e-10)
    assert math.isclose(p.velocity.twist.linear.x, .4 * math.cos(.2), abs_tol=1e-10)
    assert math.isclose(p.acceleration.accel.linear.x, -.16 * math.sin(.2), abs_tol=1e-10)
    assert p.call(p.reset, Trigger.Request()).success
    assert p.pose.pose.position.x == -8.0 and math.isclose(p.velocity.twist.linear.x, .4)
    print("PASS reference: analytic p/v/a, moving initial condition, pause, ticks and reset")


if __name__ == "__main__":
    main()
