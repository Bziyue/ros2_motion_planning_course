#!/usr/bin/env python3
"""Record real ROS sensor messages, stop the plant, replay into the SLAM stack.

The owned process groups are closed even on failure. Bags contain only /scan
and /imu/data_raw; canonical value hashes verify acquisition headers/noise.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
import rosbag2_py
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


# replay_digest_begin
def digest(message):
    # CDR alignment padding is unspecified and may differ when reserialized.
    # Compare every message value, including covariances, frame and timestamp.
    value=json.dumps(message_to_ordereddict(message),sort_keys=True,separators=(',',':'))
    return hashlib.sha256(value.encode()).hexdigest()
# replay_digest_end


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--domain',type=int,default=84)
    parser.add_argument('--output',default='tmp/appD_replay')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];out=(root/args.output).resolve()
    out.mkdir(parents=True,exist_ok=True);bag=out/'sensors'
    if bag.exists():
        raise RuntimeError('Output bag already exists; choose a new --output directory')
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    rclpy.init();node=rclpy.create_node('replay_evaluator')
    types={'/scan':LaserScan,'/imu/data_raw':Imu};received={name:{} for name in types}
    state={};tf_edges=set();processes=[];logs=[]
    sensor_qos=QoSProfile(depth=2000,reliability=ReliabilityPolicy.BEST_EFFORT)
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,cls in types.items():
        node.create_subscription(cls,topic,
            lambda m,t=topic:received[t].__setitem__(stamp(m),digest(m)),sensor_qos)
    node.create_subscription(Odometry,'/odometry',lambda m:state.__setitem__('odom',m),200)
    node.create_subscription(OccupancyGrid,'/map',lambda m:state.__setitem__('map',m),retained)
    node.create_subscription(PointCloud2,'/map/cloud',lambda m:state.__setitem__('cloud',m),retained)
    node.create_subscription(TFMessage,'/tf',lambda m:tf_edges.update(
        (tf.header.frame_id,tf.child_frame_id) for tf in m.transforms),100)
    pause=node.create_client(SetBool,'/sim/pause')

    def start(name,command):
        log=(out/f'{name}.log').open('w');logs.append(log)
        proc=subprocess.Popen(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc);return proc

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)

    def until(condition,seconds,label):
        deadline=time.monotonic()+seconds
        while not condition() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
        assert condition(),label

    def spin(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)

    def set_pause(value):
        future=pause.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        assert future.done() and future.result().success,'Pause service failed'

    try:
        recorder=start('record',['ros2','bag','record','--use-sim-time','--disable-keyboard-controls',
                                '-o',str(bag),'--topics',*types])
        simulator=start('simulator',['ros2','run','motion2d','simulator_node','--ros-args',
            '--params-file',str(root/'ros2_ws/src/motion2d_bringup/config/ch10.yaml'),
            '-p','model:=reference','-p','start_paused:=true','-p','publish_truth_tf:=false'])
        assert pause.wait_for_service(timeout_sec=10),'No simulator service'
        until(lambda:all(any(i.node_name=='rosbag2_recorder' for i in node.get_subscriptions_info_by_topic(t))
                         for t in types),10,'Recorder did not discover both sensors')
        set_pause(False)
        until(lambda:bool(received['/scan']) and max(received['/scan'])>=6*10**9,12,'No 6-second scan window')
        set_pause(True);spin(.3);stop(recorder);stop(simulator);spin(.3)
        assert recorder.returncode==0,'Recorder did not close cleanly'
        live={t:dict(messages) for t,messages in received.items()}
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
        assert {x.name for x in reader.get_all_topics_and_types()}==set(types),'Unexpected recorded topic'
        stored={t:{} for t in types};offsets=[];times=[]
        while reader.has_next():
            topic,data,record_ns,_send_ns=reader.read_next_ext()
            m=deserialize_message(data,types[topic]);ns=stamp(m)
            assert digest(m)==live[topic].get(ns),'Bag changed or lost a captured sensor payload'
            stored[topic][ns]=digest(m);times.append(record_ns);offsets.append(abs(record_ns-ns)*1e-9)
        assert len(stored['/scan'])>=55 and len(stored['/imu/data_raw'])>=1100,'Too few recorded samples'
        assert 0<min(times)<=max(times)<10*10**9,'Storage timestamps are not simulation time'
        del reader
        for messages in received.values():messages.clear()
        state.clear();tf_edges.clear()
        until(lambda:not node.get_publishers_info_by_topic('/clock'),5,'Simulator clock still present')
        replay=start('slam',['ros2','launch','motion2d_bringup','replay_slam.launch.py','rviz:=false'])
        required={'lidar_odometry','fusion','estimated_odometry','slam'}
        until(lambda:required.issubset(set(node.get_node_names())),10,'Replay estimators did not start')
        for name in required:
            topics=node.get_subscriber_names_and_types_by_node(name,'/')
            assert not any(t.startswith(('/ground_truth','/sim/','/scene/')) for t,_ in topics),(name,topics)
        player=start('play',['ros2','bag','play',str(bag),'--clock','200','--rate','.5','--delay','1',
                             '--disable-keyboard-controls','--progress-bar-update-rate','0'])
        until(lambda:len(node.get_publishers_info_by_topic('/clock'))==1,5,'Expected one player clock')
        until(lambda:player.poll() is not None,25,'Bag player timed out')
        assert player.returncode==0,'Bag player failed'
        spin(.4)
        for topic in types:
            assert received[topic]==stored[topic],f'Replay changed or dropped {topic} samples'
        assert set(state)=={'odom','map','cloud'},'SLAM output missing'
        assert state['cloud'].width>100 and any(v>=0 for v in state['map'].data),'Empty observed map'
        assert state['odom'].header.frame_id=='odom' and state['map'].header.frame_id=='map'
        assert stamp(state['odom'])>=max(stored['/scan'])-200000000,'SLAM stopped before bag end'
        assert {('map','odom'),('odom','base_link')}.issubset(tf_edges),'Missing estimated TF edges'
        assert not node.get_publishers_info_by_topic('/ground_truth/odometry'),'Live plant still running'
        result={'recorded_scan':len(stored['/scan']),'recorded_imu':len(stored['/imu/data_raw']),
                'storage_start_s':min(times)*1e-9,'storage_end_s':max(times)*1e-9,
                'max_storage_header_offset_s':max(offsets),'all_sensor_values_equal':True,
                'last_odometry_s':stamp(state['odom'])*1e-9,'map_cloud_points':state['cloud'].width,
                'observed_cells':sum(v>=0 for v in state['map'].data),'rate':.5}
        (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print('PASS sensor-only bag -> IMU/lidar SLAM:',json.dumps(result),flush=True)
        stop(replay)
    finally:
        for proc in reversed(processes):stop(proc)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
