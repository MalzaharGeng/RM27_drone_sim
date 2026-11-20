"""真实 PX4/Gazebo 回正链路检查；外力仅用于制造初始侧翻，不承担回正。"""
import argparse,csv,json,math,os,pathlib,signal,subprocess,sys,time,threading
for pkg in ['gz_transport_vendor','gz_msgs_vendor']:
    sys.path.insert(0,'/opt/ros/lyrical/opt/'+pkg+'/lib/python')
from gz.transport import Node
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_wrench_pb2 import EntityWrench
from gz.msgs.entity_pb2 import Entity
from gz.msgs.actuators_pb2 import Actuators
from gz.msgs.world_control_pb2 import WorldControl
from pymavlink import mavutil

ap=argparse.ArgumentParser();ap.add_argument('--repo',required=True);ap.add_argument('--out',required=True);ap.add_argument('--instance',type=int,default=7);ap.add_argument('--torque',type=float,default=0.7);ap.add_argument('--pulse',type=float,default=.22);args=ap.parse_args()
P=pathlib.Path(args.repo);B=P/'build/px4_sitl_default';D=pathlib.Path(args.out).resolve();D.mkdir(parents=True,exist_ok=False)
partition='flip_check_'+str(os.getpid());os.environ['GZ_PARTITION']=partition;os.environ['HEADLESS']='1';os.environ['PX4_GZ_MODEL_NAME']='x500_check';os.environ['PX4_SIM_MODEL']='gz_x500';os.environ['PX4_GZ_STANDALONE']='1'
ROOT=D/'rootfs';ROOT.mkdir();(ROOT/'etc').symlink_to(B/'etc',target_is_directory=True);(ROOT/'test_data').symlink_to(P/'test_data',target_is_directory=True)
node=Node();procs=[];records=[];motors=[];events=[];mavrows=[];pose={};linkids={};t0=time.monotonic();result={};files=[]
def now():return time.monotonic()-t0
def event(name,**kw):
    row={'t':round(now(),4),'event':name,**kw};events.append(row);print(json.dumps(row),flush=True)
def pose_cb(msg):
    for p in msg.pose:
        if p.name=='x500_check':
            q=p.orientation;v=max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y)))
            row=[now(),msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,p.position.x,p.position.y,p.position.z,q.w,q.x,q.y,q.z,math.degrees(math.acos(v))]
            records.append(row);pose.update(tilt=row[-1],z=row[4],t=now())
        linkids[p.name]=p.id
def motor_cb(msg):motors.append([now(),*msg.velocity])
node.subscribe(Pose_V,'/world/default/dynamic_pose/info',pose_cb)
node.subscribe(Actuators,'/x500_check/command/motor_speed',motor_cb)
pub=node.advertise('/world/default/wrench/persistent',EntityWrench)
clear=node.advertise('/world/default/wrench/clear',Entity)
def start(cmd,log,**kw):
    f=(D/log).open('w');files.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,**kw);procs.append(p);return p
px4=None;conn=None
def console(s):
    px4.stdin.write(s+'\n');px4.stdin.flush();event('console',command=s)
def pump(dt):
    end=time.monotonic()+dt
    while time.monotonic()<end:
        if conn:
            m=conn.recv_match(blocking=False)
            if m and m.get_type() in ['HEARTBEAT','ATTITUDE','STATUSTEXT','PARAM_VALUE','COMMAND_ACK']:
                d=m.to_dict();d['t']=now();mavrows.append(d)
        time.sleep(.005)
def safe_cleanup():
    if px4 and px4.poll() is None:
        try:console('param set FLIP_EN 0');console('commander mode manual');pump(.5)
        except Exception:pass
    for p in reversed(procs):
        if p.poll() is None:
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
    for p in reversed(procs):
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
            except ProcessLookupError:pass
    result['cleanup']={'pids':[p.pid for p in procs],'all_exited':all(p.poll() is not None for p in procs)}
try:
    gz=start(['gz','sim','-r','-s','--verbose=1',str(P/'Tools/simulation/gz/worlds/default.sdf')],'gz.log',stdin=subprocess.DEVNULL)
    event('gz_started',pid=gz.pid,partition=partition)
    factory=EntityFactory();factory.name='x500_check';factory.allow_renaming=False
    factory.sdf='<sdf version="1.9"><include><uri>file://'+str(P/'Tools/simulation/gz/models/x500/model.sdf')+'</uri><pose>0 0 0.25 0 0 0</pose></include></sdf>'
    spawned=False
    for _ in range(25):
        ok,response=node.request('/world/default/create',factory,EntityFactory,Boolean,1000)
        if ok and response.data:spawned=True;break
        if gz.poll() is not None:raise RuntimeError('Gazebo exited')
        time.sleep(.3)
    assert spawned,'spawn failed';event('spawn_upright')
    px4=start([str(B/'bin/px4'),'-i',str(args.instance),'-w',str(ROOT),'-s',str(B/'etc/init.d-posix/rcS')],'px4.log',stdin=subprocess.PIPE,text=True,cwd=ROOT)
    event('px4_started',pid=px4.pid)
    conn=mavutil.mavlink_connection('udpin:127.0.0.1:'+str(14540+args.instance),source_system=255)
    heartbeat=conn.wait_heartbeat(timeout=40);assert heartbeat,'no PX4 heartbeat'
    event('heartbeat',system=heartbeat.get_srcSystem(),mode=heartbeat.custom_mode)
    pump(18)
    console('flip_recovery status');console('listener vehicle_attitude -n 1');console('listener vehicle_land_detected -n 1');pump(1)
    # FLIP_EN stays disabled until all setup torque has ended and the vehicle settles.
    console('param set FLIP_EN 0');pump(.3)
    e=EntityWrench();e.entity.name='x500_check';e.entity.type=Entity.MODEL;e.wrench.torque.x=args.torque
    time.sleep(.5);assert pub.publish(e),'wrench publish failed';event('setup_torque_on',torque=args.torque,tilt=pose.get('tilt'))
    pump(args.pulse)
    ent=Entity();ent.name=e.entity.name;ent.type=Entity.MODEL;assert clear.publish(ent)
    event('setup_torque_off',tilt=pose.get('tilt'));pump(5)
    setup_end=now();event('settled_setup',pose=dict(pose),max_tilt=max((r[-1] for r in records),default=0))
    console('param set FLIP_EN 1');console('commander mode flip');pump(1)
    recovery_start=now();console('listener flip_recovery_control -n 160 -r 20');pump(10)
    console('flip_recovery status');console('listener flip_recovery_status -n 5 -r 10');console('listener vehicle_attitude -n 1');console('listener vehicle_land_detected -n 1');pump(3)
    console('commander mode manual');pump(1)
    hbs=[m for m in mavrows if m['mavpackettype']=='HEARTBEAT'];after=[r for r in records if r[0]>=recovery_start]
    outmot=[r for r in motors if r[0]>=recovery_start]
    result.update({'partition':partition,'source_commit':subprocess.check_output(['git','-C',str(P),'rev-parse','HEAD']).decode().strip(),'setup_end_s':setup_end,'recovery_start_s':recovery_start,'setup_tilt_deg':next((r[-1] for r in records if r[0]>=setup_end),None),'entered_flip':any(((m['custom_mode']>>16)&255)==12 for m in hbs),'max_truth_tilt_deg':max((r[-1] for r in records),default=0),'final_truth_tilt_deg':after[-1][-1] if after else None,'motor_messages':len(outmot),'max_motor_command':max((abs(v) for r in outmot for v in r[1:]),default=0),'truth_samples':len(records)})
except Exception as ex:
    result['error']=str(ex);event('error',detail=str(ex))
finally:
    safe_cleanup()
    for f in files:f.close()
    for filename,header,rows in [('truth.csv',['wall_s','sim_s','x','y','z','qw','qx','qy','qz','tilt_deg'],records),('motor_command.csv',['wall_s','motor0','motor1','motor2','motor3'],motors)]:
        with (D/filename).open('w') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
    (D/'entities.json').write_text(json.dumps(linkids,indent=2)+'\n')
    (D/'events.json').write_text(json.dumps(events,indent=2)+'\n');(D/'mavlink.json').write_text(json.dumps(mavrows,indent=2)+'\n')
    (D/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
sys.exit(1 if 'error' in result else 0)
