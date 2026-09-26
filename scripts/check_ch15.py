#!/usr/bin/env python3
"""Observed-map preview; convergence does not authorize actuation."""
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from check_ch04 import Probe

def main():
    rclpy.init();p=Probe('ideal');state={};qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    p.node.create_subscription(Path,'/plan/optimized_preview',lambda m:state.__setitem__('path',m),qos)
    p.node.create_subscription(String,'/plan/optimization_status',lambda m:state.__setitem__('status',m.data),qos)
    pub=p.node.create_publisher(PoseStamped,'/goal_pose',1)
    try:
        assert p.call(p.pause,SetBool.Request(data=True)).success
        assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:p.stamp()==0)
        for _ in range(100): assert p.call(p.step,Trigger.Request()).success
        goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=-7.5;goal.pose.position.y=-8.;goal.pose.orientation.w=1.
        pub.publish(goal);p.wait(lambda:bool(state.get('path',Path()).poses) and 'preview_only' in state.get('status',''))
        path=state['path'];assert path.header.frame_id=='map'
        assert 'preview_only' in state['status'];assert len(path.poses)>2
        assert abs(path.poses[0].pose.position.x+8)<1e-8
        assert abs(path.poses[-1].pose.position.x+7.5)<1e-8
        assert p.node.count_publishers('/plan/trajectory')==0
        for _ in range(20):assert p.call(p.step,Trigger.Request()).success
        assert p.pose.pose.position.x==-8
        result=state['status'];count=len(path.poses)
        goal.header.frame_id='bad';pub.publish(goal);p.wait(lambda:not state['path'].poses)
        assert p.call(p.reset,Trigger.Request()).success;p.wait(lambda:p.stamp()==0)
        assert not state['path'].poses
        names=[(name,ns) for name,ns in p.node.get_node_names_and_namespaces() if name=='planner']
        assert names
        topics=[t for t,_ in p.node.get_subscriber_names_and_types_by_node(*names[0])]
        assert not any('ground_truth' in t or '/world' in t for t in topics)
        print(f'PASS ch15: {count} observed preview points, {result}; endpoints, no execution, invalid goal cleanup, reset, input isolation')
    finally:p.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
