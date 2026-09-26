#!/usr/bin/env python3
"""One isolated MPC tracker with an intentionally tiny solve budget, no simulator."""
import math
import time
import rclpy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import WrenchStamped
from motion2d_interfaces.msg import MpcStatus
from rclpy.qos import QoSProfile,DurabilityPolicy

def main():
    rclpy.init();n=rclpy.create_node('check_mpc_failure');seen={}
    clock=n.create_publisher(Clock,'/clock',10);odom=n.create_publisher(Odometry,'/odometry',100)
    n.create_subscription(WrenchStamped,'/command/wrench',lambda m:seen.__setitem__('force',m),100)
    n.create_subscription(MpcStatus,'/control/mpc_status',lambda m:seen.__setitem__('qp',m),100)
    n.create_subscription(Path,'/control/prediction',lambda m:seen.__setitem__('prediction',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def wait(f):
        end=time.monotonic()+5
        while not f() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        assert f(),seen
    try:
        wait(lambda:odom.get_subscription_count()==1)
        c=Clock();c.clock.sec=1;clock.publish(c)
        for _ in range(10):rclpy.spin_once(n,timeout_sec=.01)
        m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link';m.header.stamp=c.clock
        m.pose.pose.orientation.z=math.sin(math.pi/4);m.pose.pose.orientation.w=math.cos(math.pi/4);m.twist.twist.linear.x=.2
        odom.publish(m);wait(lambda:all(k in seen for k in ['force','qp','prediction']))
        assert seen['qp'].status=='time_limit' and seen['qp'].iterations==0
        assert math.isnan(seen['qp'].max_violation)
        assert not seen['prediction'].poses,'Failure must clear old prediction'
        assert abs(seen['force'].wrench.force.x)<1e-12 and abs(seen['force'].wrench.force.y+.4)<1e-12
        assert seen['force'].header.stamp.sec==1
        print('PASS MPC ROS failure: time_limit, no fake zero residual, empty prediction, fresh body-to-odom damping force')
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
