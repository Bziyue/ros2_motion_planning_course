#!/usr/bin/env python3
"""Ch19 atomic reference test. Run a standalone navigation-source tracker first."""
import copy
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped, Point, WrenchStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty, Int64, String
from motion2d_interfaces.msg import NavigationReference, ConvexRegion2D, QuinticPiece2D
from rclpy.qos import QoSProfile, DurabilityPolicy


def quintic(boundary, end, duration):
    """Independent boundary solve, ascending coefficients in seconds."""
    c = np.zeros((2, 6)); c[:, :3] = np.array(boundary).T / [1., 1., 2.]
    t = duration
    matrix = np.array([[t**3, t**4, t**5], [3*t*t, 4*t**3, 5*t**4], [6*t, 12*t*t, 20*t**3]])
    rhs = np.array([end-c[:, 0]-t*c[:, 1]-t*t*c[:, 2], -c[:, 1]-2*t*c[:, 2], -2*c[:, 2]])
    c[:, 3:] = np.linalg.solve(matrix, rhs).T
    return c


def sample(c, t):
    return np.array([[np.polynomial.polynomial.polyval(t, np.polynomial.polynomial.polyder(row, k)) for row in c] for k in range(3)])


def main():
    rclpy.init(); node = rclpy.create_node('check_navigation_boundary'); state = {}; acknowledgements = []
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock = node.create_publisher(Clock, '/clock', 10)
    odom = node.create_publisher(Odometry, '/odometry', 100)
    plans = node.create_publisher(NavigationReference, '/navigation/reference', 1)
    stop = node.create_publisher(Empty, '/navigation/stop', 1)
    def collect(key, message): state[key] = message
    node.create_subscription(Int64, '/control/accepted_reference', lambda m: acknowledgements.append(m.data), 10)
    node.create_subscription(String, '/control/status', lambda m: collect('status', m.data), retained)
    node.create_subscription(Path, '/control/reference_path', lambda m: collect('path', m), retained)
    node.create_subscription(Odometry, '/control/reference', lambda m: collect('reference', m), 100)
    node.create_subscription(AccelStamped, '/control/reference_acceleration', lambda m: collect('acceleration', m), 100)
    node.create_subscription(WrenchStamped, '/command/wrench', lambda m: collect('command', m), 100)
    def spin(seconds=.07):
        end = time.monotonic()+seconds
        while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.01)
    def wait(predicate):
        end = time.monotonic()+5
        while not predicate() and time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.01)
        assert predicate(), state.get('status')
    def at(t, values=None):
        ns = round(t*1e9); message = Clock(); message.clock.sec = ns//10**9; message.clock.nanosec = ns%10**9
        clock.publish(message); spin(.025)
        m = Odometry(); m.header.frame_id = 'odom'; m.child_frame_id = 'base_link'; m.header.stamp = message.clock; m.pose.pose.orientation.w = 1.
        if values is not None:
            m.pose.pose.position.x, m.pose.pose.position.y = map(float, values[0]); m.twist.twist.linear.x, m.twist.twist.linear.y = map(float, values[1])
        odom.publish(m); spin()
    def plan(c, start, duration):
        message = NavigationReference(); message.motion.header.frame_id = 'odom'
        ns = round(start*1e9); message.motion.start_time.sec = ns//10**9; message.motion.start_time.nanosec = ns%10**9
        piece = QuinticPiece2D(); piece.duration = duration; piece.x = c[0].tolist(); piece.y = c[1].tolist(); message.motion.pieces = [piece]
        message.regions = [ConvexRegion2D(vertices=[Point(x=x, y=y) for x, y in [(-1., -1.), (2., -1.), (2., 1.), (-1., 1.)]])]
        return message
    def assert_reference(expected):
        ref = state['reference']; acc = state['acceleration']
        got = [[ref.pose.pose.position.x, ref.pose.pose.position.y], [ref.twist.twist.linear.x, ref.twist.twist.linear.y], [acc.accel.linear.x, acc.accel.linear.y]]
        np.testing.assert_allclose(got, expected, atol=1e-10)
    try:
        wait(lambda: odom.get_subscription_count()==1 and plans.get_subscription_count()==1)
        at(0.)
        c1 = quintic(np.zeros((3, 2)), np.array([.5, 0.]), 2.)
        first = plan(c1, .1, 2.); plans.publish(first); wait(lambda: acknowledgements == [100000000])
        at(.05); assert_reference(np.zeros((3, 2)))
        at(.5, sample(c1, .4)); assert_reference(sample(c1, .4))
        c2 = quintic(sample(c1, .6), np.array([.8, .2]), 3.)
        second = plan(c2, .7, 3.); plans.publish(second); wait(lambda: acknowledgements == [100000000, 700000000])
        plans.publish(second); wait(lambda: state.get('status', '').startswith('navigation_reference_rejected:'))
        at(.6, sample(c1, .5)); assert_reference(sample(c1, .5))
        at(.7, sample(c1, .6)); assert_reference(sample(c1, .6))
        at(.8, sample(c2, .1)); assert_reference(sample(c2, .1))
        bad = copy.deepcopy(first); bad.motion.start_time.nanosec = 900000000
        plans.publish(bad); wait(lambda: 'discontinuous_handover' in state.get('status', ''))
        at(.82, sample(c2, .12)); assert_reference(sample(c2, .12))
        stop.publish(Empty()); wait(lambda: state.get('status')=='navigation_stopped_braking' and not state['path'].poses)
        spin(); force = [state['command'].wrench.force.x, state['command'].wrench.force.y]
        np.testing.assert_allclose(force, -2*sample(c2, .12)[1], atol=1e-10)
        plans.publish(second); wait(lambda: 'stationary restart' in state.get('status', ''))
        assert acknowledgements == [100000000, 700000000]
        print('PASS atomic navigation: future ACK, old prefix, C2 p/v/a handover, invalid retention, explicit fresh brake, moving restart rejection')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
