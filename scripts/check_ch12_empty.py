#!/usr/bin/env python3
"""Run against only esdf_node: all-unknown, free exterior padding and invalid-map clearing."""
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

rclpy.init(); n=rclpy.create_node("check_ch12_empty"); state={}
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub=n.create_publisher(OccupancyGrid,"/map",qos)
for topic,kind,key in [("/esdf/cloud",PointCloud2,"cloud"),("/esdf/gradients",MarkerArray,"arrows"),("/esdf/status",String,"status")]:
    n.create_subscription(kind,topic,lambda m,k=key: state.__setitem__(k,m),qos)
def wait(fn):
    end=time.monotonic()+5
    while not fn() and time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.01)
    assert fn(),state.get("status")
try:
    wait(lambda: pub.get_subscription_count()==1)
    m=OccupancyGrid(); m.header.frame_id="map"; m.header.stamp.sec=1
    m.info.width=m.info.height=8; m.info.resolution=.1; m.info.origin.orientation.w=1.; m.data=[-1]*64
    pub.publish(m); wait(lambda: "status" in state and "arrows" in state and "cloud" in state)
    c=state["cloud"]; assert not c.is_dense and state["status"].data=="no_finite_field"
    assert np.all(np.isneginf(np.frombuffer(c.data,dtype="<f4").reshape(-1,4)[:,3]))
    assert not state["arrows"].markers[0].points
    m.header.stamp.sec=2; m.data=[0]*64; pub.publish(m)
    wait(lambda: state["cloud"].header.stamp.sec==2 and state["status"].data=="ready")
    d=np.frombuffer(state["cloud"].data,dtype="<f4").reshape(8,8,4)[:,:,3]
    assert abs(d[0,0]-.1)<1e-7 and abs(d[3,3]-.4)<1e-7
    m.info.origin.orientation.z=.1; pub.publish(m)
    wait(lambda: state["status"].data=="invalid_map" and not state["cloud"].data)
    wait(lambda: state["arrows"].markers[0].action==3)
    print("PASS all-unknown -inf with no arrows, metric exterior padding, invalid map clears cloud/markers")
finally:
    n.destroy_node(); rclpy.shutdown()
