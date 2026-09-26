#!/usr/bin/env python3
"""Nonaligned polynomial start/knot/finish; independent 6x6 boundary solve oracle."""
import copy
import math
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from motion2d_interfaces.msg import Trajectory2D, QuinticPiece2D
from check_ch04 import Probe
from check_ch06 import stamp, values
from check_ch06_time import wall_range, grid


def piece(a,b,T):
    """Rows are endpoint p/v/a; dense solve does not reuse the course coefficient formula."""
    M=np.zeros((6,6))
    for endpoint,t in enumerate([0,T]):
        for derivative in range(3):
            for k in range(derivative,6):
                M[3*endpoint+derivative,k]=math.factorial(k)/math.factorial(k-derivative)*t**(k-derivative)
    C=np.linalg.solve(M,np.vstack([a,b]))
    msg=QuinticPiece2D();msg.duration=T;msg.x=C[:,0].tolist();msg.y=C[:,1].tolist()
    return msg,C


def main():
    rclpy.init();p=Probe('trajectory');state={};imus=[];scans=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    p.node.create_subscription(String,'/sim/trajectory_status',lambda m:state.__setitem__('status',m.data),qos)
    def receive(items,m):
        if stamp(m)==0:items.clear()
        if items or stamp(m)==0:items.append(m)
    p.node.create_subscription(Imu,'/imu/data_raw',lambda m:receive(imus,m),qos_profile_sensor_data)
    p.node.create_subscription(LaserScan,'/scan',lambda m:receive(scans,m),qos_profile_sensor_data)
    pub=p.node.create_publisher(Trajectory2D,'/plan/trajectory',1)
    a=np.zeros((3,2));b=np.array([[.4,.3],[.2,-.1],[.3,.2]]);c=np.array([[.8,0],[0,0],[0,0]])
    msg=Trajectory2D();msg.header.frame_id='odom';msg.yaw=.4;msg.start_time.nanosec=203_000_000
    first,C1=piece(a,b,.458);second,C2=piece(b,c,.637);msg.pieces=[first,second]
    def expected(time):
        if time<.203:return np.zeros(2),np.zeros(2),np.zeros(2)
        local=time-.203
        if local>1.095:return c[0],np.zeros(2),np.zeros(2)
        if local<.458:C=C1
        else:C=C2;local-=.458
        return tuple(sum((math.factorial(k)/math.factorial(k-d))*local**(k-d)*C[k] for k in range(d,6)) for d in range(3))
    try:
        assert p.call(p.pause,SetBool.Request(data=True)).success
        p.spin(.2);assert p.call(p.reset,Trigger.Request()).success
        p.wait(lambda:len(imus)==len(scans)==1 and pub.get_subscription_count()==1)
        pub.publish(msg);p.wait(lambda:state['status']=='accepted')
        error=0.
        for tick in range(1,301):
            if tick==100:
                replacement=copy.deepcopy(msg);replacement.start_time.sec=1
                pub.publish(replacement);p.wait(lambda:state['status'].startswith('rejected: busy'))
            assert p.call(p.step,Trigger.Request()).success
            pos,vel,acc=expected(tick*.005)
            measured=np.array([[p.pose.pose.position.x,p.pose.pose.position.y],
                [p.velocity.twist.linear.x,p.velocity.twist.linear.y],
                [p.acceleration.accel.linear.x,p.acceleration.accel.linear.y]])
            error=max(error,np.max(np.abs(measured-np.array([pos,vel,acc]))))
        assert error<1e-8,error
        assert state['status']=='completed'
        assert [stamp(m) for m in imus]==[grid(i,137) for i in range(206)]
        assert [stamp(m) for m in scans]==[grid(i,7) for i in range(11)]
        imu_error=0.;scan_error=0.
        for m in imus:
            _,_,acc=expected(stamp(m)*1e-9)
            ca,sa=math.cos(.4),math.sin(.4)
            target=[0,0,0,ca*acc[0]+sa*acc[1],-sa*acc[0]+ca*acc[1],9.81]
            imu_error=max(imu_error,max(abs(x-y) for x,y in zip(values(m),target)))
            assert m.orientation_covariance[0]==-1
        for m in scans:
            pos,_,_=expected(stamp(m)*1e-9)
            for i,measured in enumerate(m.ranges):
                target=wall_range(*pos,.4-math.pi+i*2*math.pi/90)
                scan_error=max(scan_error,abs(measured-target))
        assert imu_error<1e-8 and scan_error<2e-6,(imu_error,scan_error)
        counts=len(imus),len(scans);p.spin(.15);assert counts==(len(imus),len(scans))
        print(f'PASS 206 IMUs/11 scans: start .203s, knot .661s, finish 1.298s; max state={error:.3g}, '
              f'IMU={imu_error:.3g}, laser={scan_error:.3g}; paused grids unchanged')
        assert p.call(p.reset,Trigger.Request()).success
        p.wait(lambda:p.stamp()==0 and state['status']=='idle')
        invalid=copy.deepcopy(msg);invalid.pieces[0].x[0]=1.
        pub.publish(invalid);p.wait(lambda:state['status'].startswith('rejected:'))
        assert p.pose.pose.position.x==0
        # A curve through the room wall must freeze before crossing it.
        finish=np.array([[20,0],[0,0],[0,0]])
        unsafe,_=piece(a,finish,.2);msg.pieces=[unsafe];msg.start_time.nanosec=0
        pub.publish(msg);p.wait(lambda:state['status']=='accepted')
        for _ in range(50):
            if not p.call(p.step,Trigger.Request()).success:break
        p.wait(lambda:p.status=='collision_predicted' and state['status']=='collision_predicted')
        assert p.pose.pose.position.x<9.8
        frozen=p.stamp();counts=len(imus),len(scans);p.spin(.1)
        assert p.stamp()==frozen and counts==(len(imus),len(scans))
        print(f'PASS curved sweep collision: frozen at {frozen:.3f}s before wall, no later samples')
    finally:
        p.node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
