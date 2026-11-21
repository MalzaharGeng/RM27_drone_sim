"""Isolated PX4/Gazebo recovery trial. No external wrench is applied."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

for lib in ("gz_transport_vendor", "gz_msgs_vendor"):
    sys.path.insert(0, f"/opt/ros/lyrical/opt/{lib}/lib/python")
from gz.transport import Node
from gz.msgs.actuators_pb2 import Actuators
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.msgs.pose_v_pb2 import Pose_V


DEFAULT = dict(EN=1, MODE=0, ENTRY_ANG=75, STOP_ANG=60, THR_HIGH=.2,
               THR_LOW=.1, LOW_MODE=0, SLEW=.5, TIME=3, MIN_PROG=3,
               PROG_T=.8, EXIT_MODE=2, RC_CONFIRM=0, OUT_DIRECT=0)
RANGES = dict(EN=(0, 1), MODE=(0, 1), ENTRY_ANG=(60, 180), STOP_ANG=(0, 120),
              THR_HIGH=(.02, .6), THR_LOW=(0, .6), LOW_MODE=(0, 3),
              SLEW=(.05, 2), TIME=(.2, 5), MIN_PROG=(0, 30), PROG_T=(.2, 2),
              EXIT_MODE=(0, 8), RC_CONFIRM=(0, 6), OUT_DIRECT=(0, 1))


def initial_rpy(tilt, axis, yaw):
    """World-Z heading composed with rotation about a horizontal body axis."""
    t, a, h = map(math.radians, (tilt / 2, axis, yaw / 2))
    ca, sa = math.cos(a), math.sin(a)
    w, x, y, z = (math.cos(h)*math.cos(t), math.sin(t)*(math.cos(h)*ca-math.sin(h)*sa),
                   math.sin(t)*(math.cos(h)*sa+math.sin(h)*ca), math.sin(h)*math.cos(t))
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def terminate(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=4)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--instance", type=int, required=True)
    ap.add_argument("--tilt", type=float, default=180)
    ap.add_argument("--axis", type=float, default=0)
    ap.add_argument("--yaw", type=float, default=0)
    ap.add_argument("--height", type=float, default=.38)
    ap.add_argument("--params", type=json.loads, default={})
    ap.add_argument("--world", type=Path)
    ap.add_argument("--model", type=Path)
    args = ap.parse_args()
    for name, val in args.params.items():
        lo, hi = RANGES[name]
        if not lo <= val <= hi or (name in {"EN","MODE","LOW_MODE","EXIT_MODE","RC_CONFIRM","OUT_DIRECT"}
                                  and int(val) != val):
            ap.error(f"invalid {name}={val}")
    params = DEFAULT | args.params
    if params["ENTRY_ANG"] <= params["STOP_ANG"]:
        ap.error("ENTRY_ANG must exceed STOP_ANG")
    repo, out = args.repo.resolve(), args.out.resolve()
    build = repo / "build/px4_sitl_default"
    out.mkdir(parents=True)
    root = out / "rootfs"
    (root / "etc/logging").mkdir(parents=True)
    for item in (build / "etc").iterdir():
        if item.name != "logging":
            (root / "etc" / item.name).symlink_to(item)
    (root / "test_data").symlink_to(repo / "test_data")
    (root / "etc/logging/logger_topics.txt").write_text(
        "flip_recovery_control 0\nflip_recovery_status 0\nvehicle_attitude 20\n"
        "vehicle_angular_velocity 20\nvehicle_status 0\nvehicle_land_detected 0\n"
        "actuator_armed 0\nesc_status 20\nactuator_motors 20\n")
    part = f"flip_sweep_{uuid.uuid4().hex[:12]}"
    os.environ["GZ_PARTITION"] = part
    os.environ["GZ_DISCOVERY_MSG_PORT"] = str(16800 + 2*args.instance)
    os.environ["GZ_DISCOVERY_SRV_PORT"] = str(16801 + 2*args.instance)
    base_env = os.environ | dict(HEADLESS="1", PX4_GZ_MODEL_NAME="x500_sweep",
                                 PX4_SIM_MODEL="gz_x500", PX4_GZ_STANDALONE="1")
    base_env = {k:v for k,v in base_env.items() if not k.startswith("PX4_PARAM_")}
    pxenv = base_env | {f"PX4_PARAM_FLIP_{k}":str(v) for k,v in params.items()}
    pxenv |= dict(PX4_PARAM_SDLOG_MODE="1", PX4_PARAM_UXRCE_DDS_CFG="0",
                  PX4_PARAM_SIM_GZ_EN="1", PX4_PARAM_MAV_BROADCAST="0")
    for k in ("PX4_GZ_MODEL_POSE", "PX4_GZ_WORLD", "PX4_SYS_AUTOSTART"):
        pxenv.pop(k, None)
    world = (args.world or repo / "Tools/simulation/gz/worlds/default.sdf").resolve()
    model = (args.model or repo / "Tools/simulation/gz/models/x500/model.sdf").resolve()
    node = Node()
    truth, motor, phases, procs, files = [], [], [], [], []
    phase = ["boot"]
    t0 = time.monotonic()
    last_motor = [0., None]

    def pose_cb(msg):
        for p in msg.pose:
            if p.name != "x500_sweep":
                continue
            q, xyz = p.orientation, p.position
            sim = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            tilt = math.degrees(math.acos(max(-1., min(1., 1-2*(q.x*q.x+q.y*q.y)))))
            truth.append((sim, phase[0], xyz.x, xyz.y, xyz.z, q.w, q.x, q.y, q.z, tilt))
            break

    def motor_cb(msg):
        if len(msg.velocity) != 4:
            return
        v = tuple(msg.velocity)
        wall = time.monotonic()
        # Keep transitions and a 50 Hz sample, even if the old bridge emits duplicates.
        key = phase[0], tuple((a > .5) - (a < -.5) for a in v)
        if wall-last_motor[0] >= .020 or key != last_motor[1]:
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            motor.append((truth[-1][0] if truth else 0., stamp, phase[0], *v))
            last_motor[:] = wall, key

    def mark(name):
        phase[0] = name
        phases.append(dict(phase=name, wall_s=time.monotonic()-t0,
                           sim_s=truth[-1][0] if truth else None))

    def launch(cmd, filename, **kwargs):
        log = (out / filename).open("w")
        files.append(log)
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, **kwargs)
        procs.append(p)
        return p

    def wait(check, timeout, why):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if check():
                return
            if any(p.poll() is not None for p in procs):
                raise RuntimeError(f"process exited while waiting for {why}")
            time.sleep(.05)
        raise RuntimeError(f"timeout waiting for {why}")

    def send(*commands):
        px.stdin.write("\n".join(commands)+"\n")
        px.stdin.flush()

    result = dict(partition=part, instance=args.instance, params=params, world=str(world),
                  model=str(model), requested=[args.tilt,args.axis,args.yaw,args.height])
    px = None
    try:
        node.subscribe(Pose_V, "/world/default/dynamic_pose/info", pose_cb)
        node.subscribe(Actuators, "/x500_sweep/command/motor_speed", motor_cb)
        gz = launch(["gz", "sim", "-r", "-s", "--verbose=1", str(world)], "gz.log",
                    stdin=subprocess.DEVNULL, env=base_env)
        r,p,y = initial_rpy(args.tilt,args.axis,args.yaw)
        factory = EntityFactory()
        factory.name = "x500_sweep"
        factory.sdf = (f'<sdf version="1.9"><include><uri>file://{model}</uri>'
                       f'<pose>0 0 {args.height} {r} {p} {y}</pose></include></sdf>')
        def spawn():
            ok, response = node.request("/world/default/create", factory, EntityFactory, Boolean, 700)
            return ok and response.data
        wait(spawn, 30, "spawn")
        px = launch([str(build/"bin/px4"), "-i", str(args.instance), "-w", str(root),
                     "-s", str(build/"etc/init.d-posix/rcS")], "px4.log",
                    env=pxenv, cwd=root, stdin=subprocess.PIPE, text=True)
        wait(lambda: truth and len(motor) >= 15 and truth[-1][0] >= max(11, motor[0][0]+7)
             and "pxh>" in (out/"px4.log").read_text(errors="replace"),
             45, "PX4 startup and simulator settling")
        send("flip_recovery status", "param show FLIP_*")
        t = truth[-1][0]
        wait(lambda: truth[-1][0] >= t+1, 10, "precheck")
        mark("recovery")
        send("commander mode flip")
        t = truth[-1][0]
        wait(lambda: truth[-1][0] >= t+11, 38, "recovery observation")
        send("flip_recovery status", "listener flip_recovery_status -n 1", "logger status")
        t = truth[-1][0]
        wait(lambda: truth[-1][0] >= t+.3, 5, "readback")
        mark("cleanup")
        send("logger off", "shutdown")
        try:
            px.wait(timeout=7)
        except subprocess.TimeoutExpired:
            pass
    except Exception as exc:
        result["error"] = repr(exc)
    finally:
        if px is not None:
            px.stdin.close()
        for proc in reversed(procs):
            terminate(proc)
        for log in files:
            log.close()
        for filename, columns, rows in [
            ("truth.csv", "sim_s phase x y z qw qx qy qz tilt_deg", truth),
            ("motor.csv", "sim_s stamp_s phase m0 m1 m2 m3", motor),
        ]:
            with (out/filename).open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(columns.split())
                w.writerows(rows)
        result.update(phases=phases, processes=[dict(pid=p.pid, rc=p.returncode) for p in procs],
                      pose_samples=len(truth), motor_samples=len(motor))
        (out/"meta.json").write_text(json.dumps(result, indent=2)+"\n")
        print(out.name, json.dumps(result), flush=True)
    return int("error" in result)


if __name__ == "__main__":
    sys.exit(main())
