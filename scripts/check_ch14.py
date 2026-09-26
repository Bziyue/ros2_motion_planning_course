#!/usr/bin/env python3
"""Plan from observed data, execute one timed curve, and compare p/v/a analytically."""
import copy
import csv
import math
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from motion2d_interfaces.msg import Trajectory2D
from check_ch04 import Probe
from check_ch06 import stamp


def main():
    rclpy.init(); p=Probe('trajectory'); state={}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,kind,key in [('/plan/corridor_path',Path,'route'),('/sim/trajectory_status',String,'status'),
                           ('/plan/trajectory_path',Path,'preview')]:
        p.node.create_subscription(kind,topic,lambda m,k=key:state.__setitem__(k,m),qos)
    p.node.create_subscription(Trajectory2D,'/plan/trajectory',lambda m:state.__setitem__('trajectory',m),1)
    goal=p.node.create_publisher(PoseStamped,'/goal_pose',1)
    commands=p.node.create_publisher(Trajectory2D,'/plan/trajectory',1)
    execute=p.node.create_client(Trigger,'/trajectory/execute')
    assert execute.wait_for_service(timeout_sec=5)
    try:
        assert p.call(p.pause,SetBool.Request(data=True)).success
        p.spin(.2);assert p.call(p.reset,Trigger.Request()).success
        p.wait(lambda:p.stamp()==0)
        for _ in range(100):assert p.call(p.step,Trigger.Request()).success
        msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=-7.5;msg.pose.position.y=-8.;msg.pose.orientation.w=1.
        goal.publish(msg);p.wait(lambda:'route' in state and len(state['route'].poses)==2)
        result=p.call(execute,Trigger.Request());assert result.success,result.message
        p.wait(lambda:'trajectory' in state and state.get('status',String()).data=='accepted')
        t=state['trajectory']; assert t.header.frame_id=='odom' and len(t.pieces)==1
        start=t.start_time.sec+t.start_time.nanosec*1e-9;T=t.pieces[0].duration
        assert math.isclose(start,.75) and math.isclose(T,1.25) and t.yaw==0
        assert list(t.pieces[0].y)==[-8.,0,0,0,0,0]
        expected=[-8,0,0,.5*10/T**3,-.5*15/T**4,.5*6/T**5]
        assert max(abs(a-b) for a,b in zip(t.pieces[0].x,expected))<1e-12
        p.wait(lambda:'preview' in state)
        preview=state['preview']; assert math.isclose(stamp(preview.poses[0])*1e-9,start)
        assert math.isclose(stamp(preview.poses[-1])*1e-9,start+T)
        samples=[];max_error=[0.,0.,0.]
        for _ in range(340):
            assert p.call(p.step,Trigger.Request()).success
            u=max(0.,min(1.,(p.stamp()-start)/T))
            x=-8+.5*(10*u**3-15*u**4+6*u**5)
            v=.5/T*(30*u*u-60*u**3+30*u**4)
            a=.5/T**2*(60*u-180*u*u+120*u**3)
            actual=[p.pose.pose.position.x,p.velocity.twist.linear.x,p.acceleration.accel.linear.x]
            for i,(measured,reference) in enumerate(zip(actual,[x,v,a])):max_error[i]=max(max_error[i],abs(measured-reference))
            assert abs(p.pose.pose.position.y+8)<1e-10
            samples.append([p.stamp(),x,v,a,*actual])
        assert max(max_error)<1e-9,max_error
        assert state['status'].data=='completed'
        p.spin(.15);assert state['status'].data=='completed'
        assert p.pose_pub.get_subscription_count()==p.velocity_pub.get_subscription_count()==p.wrench_pub.get_subscription_count()==0
        assert p.node.count_publishers('/clock')==1
        with open('tmp/ch14_execution.csv','w') as f:
            writer=csv.writer(f);writer.writerow(['t','x_ref','v_ref','a_ref','x','v','a']);writer.writerows(samples)
        # Invalid input must be visibly rejected and may not alter the held endpoint.
        bad=copy.deepcopy(t);bad.start_time.sec=3;bad.header.frame_id='map';commands.publish(bad)
        p.wait(lambda:state['status'].data.startswith('rejected:'))
        assert p.call(p.step,Trigger.Request()).success
        assert abs(p.pose.pose.position.x+7.5)<1e-10
        assert p.call(p.reset,Trigger.Request()).success
        p.wait(lambda:p.stamp()==0 and state['status'].data=='idle')
        for _ in range(20):assert p.call(p.step,Trigger.Request()).success
        assert p.pose.pose.position.x==-8 and state['status'].data=='idle'
        print(f'PASS observed-route execution: start={start}s T={T}s, 340 ticks, max p/v/a error={max_error}, '
              'future hold, completion, message coefficients/times, input isolation, invalid frame, reset')
    finally:
        p.node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
