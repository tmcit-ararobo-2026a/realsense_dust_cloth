#!/usr/bin/env python3
"""Live test: run Unity and real gn10 localization in the same isolated ROS domain.

The tracker consumes localization TF, never simulated ground-truth TF. This script
only sends drive and shooter commands inside the explicitly isolated test domain.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool, Float32
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--params-file',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--mode',choices=['map','depth'],default='map')
    parser.add_argument('--require-observed-landing',action='store_true')
    args=parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID']=='0':
        raise RuntimeError('Use a dedicated ROS_DOMAIN_ID for this live command test')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('map_cloth_verification')
    positions=[];landings=[];states=[];poses=[];sim=[0.0];ready=[False];tracking=['IDLE']
    def point(m,target):
        target.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,x=m.point.x,y=m.point.y,z=m.point.z))
    node.create_subscription(PointStamped,'/cloth/position',lambda m:point(m,positions),100)
    node.create_subscription(PointStamped,'/cloth/landing_position',lambda m:point(m,landings),100)
    node.create_subscription(String,'/cloth/background_status',lambda m:states.append(dict(stamp=sim[0],state=m.data)),100)
    node.create_subscription(Bool,'/cloth/ready',lambda m:ready.__setitem__(0,m.data),10)
    node.create_subscription(String,'/cloth/tracking_state',lambda m:tracking.__setitem__(0,m.data),10)
    def pose(m):
        p=m.pose.pose.position;poses.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,x=p.x,y=p.y))
    node.create_subscription(PoseWithCovarianceStamped,'/platform_constraint',pose,100)
    node.create_subscription(Clock,'/clock',lambda m:sim.__setitem__(0,m.clock.sec+m.clock.nanosec/1e9),
        QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
    command=node.create_publisher(Twist,'/robot/operation/cmd_vel',10)
    throw=node.create_publisher(Bool,'/belt/throw',10)
    speed=node.create_publisher(Float32,'/belt/speed_ratio',10)
    def advance(duration,velocity=0.):
        end=time.monotonic()+duration;next_publish=0.
        while time.monotonic()<end:
            if time.monotonic()>=next_publish:
                m=Twist();m.linear.x=velocity;command.publish(m);next_publish=time.monotonic()+.02
            rclpy.spin_once(node,timeout_sec=.005)
    def until(predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            advance(.03)
            if predicate():return True
        return False
    trials=[];result={}
    with (output/'tracker.log').open('w') as log:
        process=subprocess.Popen(['ros2','launch','realsense_dust_cloth','cloth_tracker.launch.py',
            'params_file:='+str(Path(args.params_file).resolve()),'enable_analyzer:=false','rviz:=false'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            throw.publish(Bool(data=False))
            assert until(lambda:ready[0],15), 'Background never ready; '+str(states[-5:])
            assert args.mode!='map' or len(poses)>3, 'Actual localization poses are missing'
            client=node.create_client(GetParameters,'/cloth_tracker_cuda_node/get_parameters')
            assert client.wait_for_service(timeout_sec=2)
            request=GetParameters.Request();request.names=['background_mode','use_sim_time','use_direct_realsense','use_bg_subtraction']
            future=client.call_async(request);assert until(future.done,3)
            values=future.result().values
            assert values[0].string_value==args.mode
            assert values[1].bool_value and not values[2].bool_value and values[3].bool_value
            advance(.5)
            assert len(positions)==0,'False detection before a throw'
            for index,ratio in enumerate((.4,.7,.4)):
                movement=None
                if index==2:
                    before=poses[-1] if poses else None;state_index=len(states)
                    advance(1.0,velocity=.4)
                    stop_time=sim[0];wall=time.monotonic()
                    assert until(lambda:ready[0],5),'Not ready after motion'
                    assert any(s['state']=='MOVING_OR_SETTLING' for s in states[state_index:])
                    after=poses[-1] if poses else None
                    distance=((after['x']-before['x'])**2+(after['y']-before['y'])**2)**.5 if before and after else None
                    if args.mode=='map':assert distance>.2,'Localization did not observe actual travel'
                    movement=dict(sim_wait=sim[0]-stop_time,wall_wait=time.monotonic()-wall,localized_travel_m=distance)
                assert until(lambda:ready[0],3)
                point_index=len(positions);land_index=len(landings)
                speed.publish(Float32(data=ratio));advance(.1)
                throw.publish(Bool(data=True));advance(.08);throw.publish(Bool(data=False))
                detected=until(lambda:len(positions)>point_index,3)
                ended=until(lambda:tracking[0] in ('LANDED','LOST','RESTING'),5) if detected else False
                advance(.25)
                observed=positions[point_index:];ending=landings[land_index:]
                trial=dict(trial=index+1,ratio=ratio,detected=detected,points=len(observed),landing_message=bool(ending),
                    terminal=tracking[0] if ended else 'TIMEOUT',
                    first=observed[0] if observed else None,last=observed[-1] if observed else None,
                    observed_landing=any(p['z']<=.15 for p in observed),published_landing=ending[-1] if ending else None,
                    movement=movement)
                trials.append(trial);print(json.dumps(trial),flush=True)
                advance(.5)
            logs=(output/'tracker.log').read_text()
            if args.mode=='map':
                assert 'BACKGROUND CALIBRATED' not in logs and 'FAST BACKGROUND RELEARNED' not in logs
                assert any(s['state']=='MAP_READY' for s in states)
            assert all(t['detected'] and t['points']>=5 for t in trials),'A simulated throw was not tracked'
            if args.require_observed_landing:
                assert all(t['observed_landing'] for t in trials), 'Not all flights were observed to the landing height'
            assert not any(s['state']=='MAP_PROCESSING_ERROR' for s in states)
            result=dict(passed=True,mode=args.mode,trials=trials,all_landings_observed=all(t['observed_landing'] for t in trials),localization_messages=len(poses),
                checks=['launch respects YAML','real localization input','stationary shots','travel then stop and shot',
                        'no shot-counter dependency','no depth learning in map mode'] if args.mode=='map' else
                       ['launch respects YAML','legacy background mode','stationary shots','travel then relearn and shot'])
        except Exception as error:
            result=dict(passed=False,mode=args.mode,trials=trials,error=str(error),last_states=states[-10:])
            raise
        finally:
            command.publish(Twist());throw.publish(Bool(data=False));advance(.1)
            (output/'result.json').write_text(json.dumps(result,indent=2))
            (output/'observations.json').write_text(json.dumps(dict(positions=positions,landings=landings,states=states,poses=poses),indent=2))
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
