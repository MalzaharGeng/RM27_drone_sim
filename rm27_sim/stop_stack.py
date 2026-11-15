#!/usr/bin/env python3
"""Stop the running SITL stack by exact /proc cmdline match.

Avoids the classic `pkill -f <pattern>` self-match problem when the whole
command is delivered through `bash -lc '<script text>'`: our own shell has
`-lc` as argv[1] and is skipped explicitly.
"""
import os
import signal
import subprocess

KEYS = (
    "launch_stack.sh", "MicroXRCEAgent", "make px4_sitl", "cmake --build",
    "gz_bridge", "run_stereo_slam.sh", "stereo_slam_node", "parameter_bridge",
    "sim_truth_tf.py", "sim_vio_health.py", "slam_pose_to_px4.py",
    "sim_monitor.py", "px4_square_bt.py", "sim_pilot.py",
)

r = subprocess.run(["bash", "/home/gengyinghao/drone_sim/stop_sim.sh"],
                   capture_output=True, text=True)
print("stop_sim.sh:", (r.stdout + r.stderr).strip()[:200])

me = os.getpid()
killed = []
for entry in os.listdir("/proc"):
    if not entry.isdigit():
        continue
    pid = int(entry)
    if pid == me:
        continue
    try:
        raw = open("/proc/" + entry + "/cmdline", "rb").read()
    except OSError:
        continue
    parts = [x.decode("utf-8", "replace") for x in raw.split(b"\0") if x]
    if not parts:
        continue
    exe = os.path.basename(parts[0])
    if exe in ("bash", "sh", "python3", "wsl") and any(
            a in ("-lc", "-c") for a in parts[:2]):
        continue
    joined = " ".join(parts)
    if any(k in joined for k in KEYS):
        try:
            os.kill(pid, signal.SIGKILL)
            killed.append((pid, joined[:90]))
        except OSError:
            pass
for pid, cmd in killed:
    print("killed", pid, cmd)
print("total killed", len(killed))
