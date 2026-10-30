#!/usr/bin/env python3
"""Live Unity no-throw/motion and owned-localization outage/recovery check."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PointStamped,Twist
from std_msgs.msg import Bool,Float32,String
p=argparse.ArgumentParser();p.add_argument('--params-file',required=True);p.add_argument('--localization-pid',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args()
assert os.environ.get('ROS_DOMAIN_ID')=='91','Dedicated Unity test domain required'
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('external_cloud_fault_test')
points=[];landings=[];states=[];ready=[False];tracking=['IDLE']
node.create_subscription(PointStamped,'/cloth/position',lambda m:points.append([m.point.x,m.point.y,m.point.z]),100)
node.create_subscription(PointStamped,'/cloth/landing_position',lambda m:landings.append(m.point.z),10)
node.create_subscription(String,'/cloth/background_status',lambda m:states.append(m.data),100)
node.create_subscription(String,'/cloth/tracking_state',lambda m:tracking.__setitem__(0,m.data),100)
node.create_subscription(Bool,'/cloth/ready',lambda m:ready.__setitem__(0,m.data),10)
cmd=node.create_publisher(Twist,'/robot/operation/cmd_vel',10)
throw=node.create_publisher(Bool,'/belt/throw',10);speed=node.create_publisher(Float32,'/belt/speed_ratio',10)
def run(sec,vel=0.0):
    end=time.monotonic()+sec
    while time.monotonic()<end:
        msg=Twist();msg.linear.x=float(vel);cmd.publish(msg);rclpy.spin_once(node,timeout_sec=.01)
def wait(pred,sec):
    end=time.monotonic()+sec
    while time.monotonic()<end:
        run(.02)
        if pred():return True
    return False
f=(out/'tracker.log').open('w');proc=subprocess.Popen(['ros2','launch','realsense_dust_cloth','cloth_tracker.launch.py','params_file:='+str(Path(a.params_file).resolve()),'enable_analyzer:=false'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
suspended=False;result={'passed':False}
try:
    assert wait(lambda:ready[0],15),'External input not ready'
    run(1)
    assert not points,'False tracking without a throw'
    run(.8,.35);run(.5)
    assert wait(lambda:ready[0],5),'Not ready after no-throw motion'
    run(.5)
    assert not points,'False tracking after moving without a throw'
    speed.publish(Float32(data=.4));run(.1);throw.publish(Bool(data=True));run(.06);throw.publish(Bool(data=False))
    assert wait(lambda:len(points)>=5,3),'Throw not captured'
    before=len(points);start=len(states)
    os.kill(a.localization_pid,signal.SIGSTOP);suspended=True
    run(.9)
    assert 'EXTERNAL_RESULT_TIMEOUT' in states[start:],'Missing external timeout status'
    assert not ready[0],'Ready stayed true during localization outage'
    assert tracking[0]=='LOST',f'Outage did not end tracking: {tracking[0]}'
    assert not landings,'Outage falsely published landing'
    os.kill(a.localization_pid,signal.SIGCONT);suspended=False
    assert wait(lambda:ready[0],6),'No recovery after localization resumed'
    run(.5)
    result={'passed':True,'no_throw_false_points':0,'points_before_outage':before,'terminal_during_outage':'LOST','recovered':True}
except Exception as e:
    result['error']=str(e);raise
finally:
    if suspended:os.kill(a.localization_pid,signal.SIGCONT)
    cmd.publish(Twist());throw.publish(Bool(data=False));run(.1)
    (out/'result.json').write_text(json.dumps(result,indent=2));(out/'states.json').write_text(json.dumps(states))
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    f.close();node.destroy_node();rclpy.shutdown()
print(json.dumps(result))
