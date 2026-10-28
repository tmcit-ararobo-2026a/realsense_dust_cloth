#!/usr/bin/env python3
"""Synthetic camera/TF test in an isolated domain, including paused ROS time."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Bool
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

def main():
    p=argparse.ArgumentParser();p.add_argument('--executable',required=True)
    p.add_argument('--params-file',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('map_pose_guard_test');states=[];points=[];ready=[False]
    n.create_subscription(String,'/cloth/background_status',lambda m:states.append(m.data),100)
    n.create_subscription(Bool,'/cloth/ready',lambda m:ready.__setitem__(0,m.data),100)
    n.create_subscription(PointStamped,'/cloth/position',lambda m:points.append(m),100)
    depth=n.create_publisher(Image,'/camera/camera/depth/image_rect_raw',10)
    info=n.create_publisher(CameraInfo,'/camera/camera/depth/camera_info',10)
    clock=n.create_publisher(Clock,'/clock',10);tf=TransformBroadcaster(n)
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
    def stamp(t):
        ns=round(t*1e9);c=Clock();c.clock.sec=ns//10**9;c.clock.nanosec=ns%10**9;return c.clock
    def step(t,pose=True,image_time=None):
        c=Clock();c.clock=stamp(t);clock.publish(c);spin(.05)
        camera=CameraInfo();camera.width=64;camera.height=48;camera.k=[40.,0.,32.,0.,40.,24.,0.,0.,1.]
        camera.header.stamp=stamp(t if image_time is None else image_time)
        camera.header.frame_id='camera_depth_optical_frame';info.publish(camera)
        if pose:
            messages=[]
            for dt in [0.,.1]:
                m=TransformStamped();m.header.stamp=stamp(t+dt);m.header.frame_id='map';m.child_frame_id='base_link'
                m.transform.rotation.w=1.;messages.append(m)
            tf.sendTransform(messages)
        spin(.05)
        image=Image();image.header=camera.header;image.width=64;image.height=48;image.encoding='16UC1';image.step=128
        image.data=np.full((48,64),4000,dtype=np.uint16).tobytes();depth.publish(image);spin(.5)
    with (out/'tracker.log').open('w') as log:
        proc=subprocess.Popen([a.executable,'--ros-args','--params-file',a.params_file],stdout=log,stderr=subprocess.STDOUT)
        try:
            spin(2)
            step(10.0);assert ready[0] and states[-1]=='MAP_READY',states[-5:]
            step(10.2);assert ready[0]
            wall=time.monotonic();step(10.8,pose=False)
            assert time.monotonic()-wall<1.0,'Paused /clock caused an unbounded TF wait'
            assert not ready[0] and states[-1]=='MAP_WAITING_FOR_POSE',states[-5:]
            step(11.0);assert ready[0] and states[-1]=='MAP_READY',states[-5:]
            step(12.0,pose=False,image_time=10.8)
            assert not ready[0] and states[-1]=='MAP_IMAGE_TIME_INVALID',states[-5:]
            step(12.2);assert ready[0] and not points,states[-5:]
            result=dict(passed=True,checks=['first frame without background learning','timestamped TF',
                'missing pose blocks detection','paused ROS clock has bounded wait','pose recovery','old image rejected'])
            (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
        finally:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
