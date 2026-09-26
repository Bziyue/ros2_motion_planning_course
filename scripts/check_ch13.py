#!/usr/bin/env python3
"""Observed-map corridors, independently checked by polygon/square separating axes."""
import json
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from check_ch04 import Probe
from check_ch06 import stamp


def touches(poly, low, high):
    box=np.array([low,[high[0],low[1]],high,[low[0],high[1]]])
    edges=np.roll(poly,-1,axis=0)-poly
    axes=[np.array([1.,0]),np.array([0,1.])]
    axes.extend(np.array([e[1],-e[0]])/np.linalg.norm(e) for e in edges)
    for n in axes:
        a,b=poly@n,box@n
        if max(a)<min(b)-1e-10 or max(b)<min(a)-1e-10: return False
    return True


def main():
    rclpy.init(); p=Probe("ideal"); state={}
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,kind,key in [("/planning/grid",OccupancyGrid,"grid"),("/plan/path_raw",Path,"raw"),
                           ("/plan/corridor_path",Path,"route"),("/plan/corridors",MarkerArray,"regions"),
                           ("/plan/corridor_status",String,"status")]:
        p.node.create_subscription(kind,topic,lambda m,k=key: state.__setitem__(k,m),retained)
    pub=p.node.create_publisher(PoseStamped,"/goal_pose",1)
    def goal(x,y):
        m=PoseStamped(); m.header.frame_id="map"; m.pose.position.x=x; m.pose.position.y=y; m.pose.orientation.w=1.
        state.pop("status",None); pub.publish(m)
        p.wait(lambda: "status" in state); p.spin(.1)
    try:
        assert p.call(p.pause,SetBool.Request(data=True)).success
        p.spin(.3); assert p.call(p.reset,Trigger.Request()).success
        p.wait(lambda: "grid" in state and stamp(state["grid"])==0)
        for _ in range(100): assert p.call(p.step,Trigger.Request()).success
        p.wait(lambda: stamp(state["grid"])==500_000_000)
        goal(-4.,-1.); assert state["status"].data=="found",state["status"].data
        route=state["route"]; markers=state["regions"].markers; raw=state["raw"]
        assert markers[0].action==3 and len(markers)>=3
        assert len(route.poses)==len(markers) and len(raw.poses)>len(route.poses)
        assert route.header==raw.header and route.header.frame_id=="map"
        assert all(m.header==route.header for m in markers)
        points=np.array([[p.pose.position.x,p.pose.position.y] for p in route.poses])
        np.testing.assert_allclose(points[0],[-8,-8],atol=1e-8)
        np.testing.assert_allclose(points[-1],[-4,-1],atol=1e-8)
        g=state["grid"]; r=g.info.resolution; origin=np.array([g.info.origin.position.x,g.info.origin.position.y])
        blocked=np.array(g.data).reshape(g.info.height,g.info.width)
        polygons=[]
        for i,m in enumerate(markers[1:]):
            assert m.type==m.LINE_STRIP and m.points[0]==m.points[-1] and m.id==i
            poly=np.array([[v.x,v.y] for v in m.points[:-1]]); polygons.append(poly.tolist())
            assert len(poly)>=4
            for a,b in zip(poly,np.roll(poly,-1,axis=0)):
                e=b-a; normal=np.array([e[1],-e[0]])/np.linalg.norm(e)
                assert max(points[i:i+2]@normal-normal@a)<1e-8
            lo=np.floor((poly.min(axis=0)-origin)/r).astype(int)-1
            hi=np.floor((poly.max(axis=0)-origin)/r).astype(int)+1
            for y in range(max(0,lo[1]),min(g.info.height,hi[1]+1)):
                for x in range(max(0,lo[0]),min(g.info.width,hi[0]+1)):
                    if blocked[y,x]:
                        low=origin+r*np.array([x,y])
                        assert not touches(poly,low,low+r),f"region {i} touches blocked cell {x},{y}"
        with open("tmp/ch13_observed.json","w") as f:
            json.dump(dict(resolution=r,width=g.info.width,height=g.info.height,origin=origin.tolist(),
                blocked=list(g.data),polygons=polygons,route=points.tolist(),
                raw=[[p.pose.position.x,p.pose.position.y] for p in raw.poses]),f)
        region_count=len(markers)-1; raw_count=len(raw.poses)
        goal(-7.5,-8.); assert state["status"].data=="found"
        assert len(state["regions"].markers)-1<region_count and state["regions"].markers[0].action==3
        goal(8.,8.); assert state["status"].data=="invalid_goal"
        assert len(state["regions"].markers)==1 and not state["route"].poses and not state["raw"].poses
        goal(-7.5,-8.); assert state["status"].data=="found"
        assert p.call(p.reset,Trigger.Request()).success
        p.wait(lambda: state["status"].data=="reset")
        assert len(state["regions"].markers)==1 and not state["route"].poses
        print(f"PASS {raw_count} raw A* points -> {region_count} observed-map corridors, "
              "independent SAT validation, assigned endpoints, matching headers, clear on smaller goal/failure/reset")
    finally:
        p.call(p.pause,SetBool.Request(data=True)); p.node.destroy_node(); rclpy.shutdown()


if __name__=="__main__": main()
