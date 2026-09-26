#!/usr/bin/env python3
"""Verify actual map->ESDF messages against independent nearest-centre enumeration."""
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from check_ch04 import Probe
from check_ch06 import stamp


def main():
    rclpy.init(); p = Probe("ideal"); maps = {}; clouds = {}; markers = {}; state = {}
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    p.node.create_subscription(OccupancyGrid, "/map", lambda m: maps.__setitem__(stamp(m),m), retained)
    p.node.create_subscription(PointCloud2, "/esdf/cloud", lambda m: clouds.__setitem__(stamp(m),m), retained)
    p.node.create_subscription(MarkerArray, "/esdf/gradients",
        lambda m: markers.__setitem__(stamp(m.markers[0]),m), retained)
    p.node.create_subscription(String, "/esdf/status", lambda m: state.__setitem__("status",m.data), retained)
    try:
        assert p.call(p.pause, SetBool.Request(data=True)).success
        p.spin(.3); maps.clear(); clouds.clear(); markers.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: 0 in clouds)
        for _ in range(100):
            assert p.call(p.step, Trigger.Request()).success
        t = 500_000_000
        p.wait(lambda: t in maps and t in clouds and t in markers)
        m, c = maps[t], clouds[t]
        assert c.header.frame_id == "map" and c.width == m.info.width and c.height == m.info.height
        assert [(f.name,f.offset,f.datatype) for f in c.fields] == [("x",0,7),("y",4,7),("z",8,7),("distance",12,7)]
        assert c.point_step == 16 and c.row_step == c.width*16 and len(c.data) == c.width*c.height*16
        a = np.frombuffer(c.data, dtype=">f4" if c.is_bigendian else "<f4").reshape(c.height,c.width,4)
        assert np.all(a[:,:,2] == 0) and c.is_dense
        raw = np.array(m.data).reshape(c.height,c.width); blocked = (raw < 0)|(raw > 35)
        padded = np.pad(blocked, 1, constant_values=True)
        seeds_o = np.argwhere(padded); seeds_f = np.argwhere(~padded)
        rng = np.random.default_rng(1214); worst = 0.
        for _ in range(100):
            y, x = int(rng.integers(c.height)), int(rng.integers(c.width))
            target = seeds_f if blocked[y,x] else seeds_o
            d = np.sqrt(np.min(np.sum((target-[y+1,x+1])**2,axis=1)))*m.info.resolution
            if blocked[y,x]: d = -d
            worst = max(worst, abs(float(a[y,x,3])-d))
            assert abs(a[y,x,0]-(m.info.origin.position.x+(x+.5)*m.info.resolution)) < 1e-5
            assert abs(a[y,x,1]-(m.info.origin.position.y+(y+.5)*m.info.resolution)) < 1e-5
        assert worst < 1e-5 and state["status"] == "ready"
        lines = markers[t].markers[0]; assert lines.points and len(lines.points)%6 == 0
        assert lines.header == c.header and lines.type == lines.LINE_LIST
        for i in range(0,len(lines.points),6):
            start,end = lines.points[i:i+2]
            assert 0 < np.hypot(end.x-start.x,end.y-start.y) <= .40000001
            q = (np.array([start.x,start.y])-[m.info.origin.position.x,m.info.origin.position.y])/m.info.resolution-.5
            ix,iy = np.floor(q).astype(int); u,v = q-[ix,iy]
            d00,d10,d01,d11 = a[iy,ix,3],a[iy,ix+1,3],a[iy+1,ix,3],a[iy+1,ix+1,3]
            gradient = np.array([(1-v)*(d10-d00)+v*(d11-d01),
                                 (1-u)*(d01-d00)+u*(d11-d10)])/m.info.resolution
            direction = np.array([end.x-start.x,end.y-start.y])
            if np.linalg.norm(gradient) > 1e-3:
                assert np.dot(gradient,direction)/(np.linalg.norm(gradient)*np.linalg.norm(direction)) > .995
        np.savez("tmp/ch12_observed.npz", cloud=a, resolution=m.info.resolution,
            origin=[m.info.origin.position.x,m.info.origin.position.y],
            starts=[[p.x,p.y] for p in lines.points[::6]], ends=[[p.x,p.y] for p in lines.points[1::6]])
        topics = {t for t,_ in p.node.get_subscriber_names_and_types_by_node("esdf","/")}
        assert "/map" in topics and not any(t.startswith(("/ground_truth","/sim","/tf")) for t in topics)
        first = bytes(clouds[0].data); clouds.clear()
        assert p.call(p.reset, Trigger.Request()).success
        p.wait(lambda: 0 in clouds); assert bytes(clouds[0].data) == first
        print(f"PASS {c.width}x{c.height} metric distance cloud, 100 independent probes max error {worst:.2g} m, "
              f"{len(lines.points)//6} arrows, exact map stamp, isolation and reset")
    finally:
        p.call(p.pause, SetBool.Request(data=True)); p.node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
