#!/usr/bin/env python3
"""独立真值记录器（只读，不参与控制）。

数据源：Gazebo Transport 的 /world/<world>/dynamic_pose/info（Pose_V）。
与旧 truth_odom_node 的区别（旧实现的三处缺陷都在这里修掉）：

1. 按实体名选机，不用 transforms[0] / TF 下标。
   实测 dynamic_pose/info 里 model 根节点名为 Drone-Harrier，其 position
   就是世界系（ENU）坐标；base_link/rotor_* 是相对 model 的局部坐标，不能混用。
2. 速度由「源时间 sim 秒」差分得到，不用回调接收时刻差分。
   记录里同时保留 recv_wall_us / recv_mono_us，便于事后单独统计传输延迟，
   但绝不把它们当成采样时刻。
3. 不做 EMA / P90 之类的平滑把样本推近接收时刻。
   速度只在源 dt 落在合理区间时才计算，其余保持 NaN 并如实记录。

输出 CSV：
  recv_wall_us, recv_mono_us, sim_us, name,
  x,y,z,qx,qy,qz,qw, vx,vy,vz, sim_dt
  （位置/姿态/速度均为 Gazebo 世界 ENU，重力指向 -z）

用法：
  truth_recorder.py --out truth.csv [--entity x500_mono_cam_0]
                    [--topic /world/rm27_world/dynamic_pose/info]
                    [--seconds 0]
"""
import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import time

NUM = re.compile(r"^\s*([A-Za-z_]+)\s*:\s*(.*?)\s*$")


def _f(v, default=0.0):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def parse_pose_block(lines):
    """lines: text lines of one top-level `pose { ... }` block."""
    name = ""
    p = {"x": 0.0, "y": 0.0, "z": 0.0}
    q = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    cur = None
    for line in lines:
        s = line.strip()
        if s.startswith("name:"):
            name = s.split(":", 1)[1].strip().strip('"')
        elif s == "position {":
            cur = p
        elif s == "orientation {":
            cur = q
        elif s == "}":
            cur = None
        elif cur is not None:
            m = NUM.match(s)
            if m and m.group(1) in cur:
                cur[m.group(1)] = _f(m.group(2))
    return name, p, q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--entity", default=os.environ.get("TRUTH_ENTITY", "Drone-Harrier"))
    ap.add_argument("--topic", default=os.environ.get(
        "TRUTH_GZ_TOPIC", "/world/rm27_world/dynamic_pose/info"))
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = 不限时")
    args = ap.parse_args()

    cmd = ["gz", "topic", "-e", "-t", args.topic]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, bufsize=1)

    fh = open(args.out, "w", newline="")
    wr = csv.writer(fh)
    wr.writerow(["recv_wall_us", "recv_mono_us", "sim_us", "name",
                 "x", "y", "z", "qx", "qy", "qz", "qw",
                 "vx", "vy", "vz", "sim_dt"])

    stamp = None          # (sec, nsec) of current message header
    in_header = False
    header_sec = None
    header_nsec = None
    block = None          # list of lines for current pose
    last = None           # (sim_us, x, y, z)
    rows = 0
    stop = {"flag": False}

    def on_term(_sig, _frm):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    start = time.monotonic()
    try:
        for line in proc.stdout:
            if stop["flag"]:
                break
            if args.seconds and time.monotonic() - start > args.seconds:
                break
            s = line.rstrip("\n")
            stripped = s.strip()
            if s == "header {":
                in_header = True
                header_sec = None
                header_nsec = None
                continue
            if in_header:
                if stripped == "}":
                    in_header = False
                    continue
                m = NUM.match(stripped)
                if m:
                    if m.group(1) == "sec":
                        header_sec = _f(m.group(2))
                    elif m.group(1) == "nsec":
                        header_nsec = _f(m.group(2))
                continue
            if s == "pose {":
                block = []
                continue
            if block is not None:
                if s == "}":
                    name, p, q = parse_pose_block(block)
                    block = None
                    if name != args.entity:
                        continue
                    if header_sec is None:
                        # 没拿到 header 就不编造时间
                        continue
                    # protobuf 文本省略 0 值字段：nsec 为 0 时整行不出现，按 0 处理。
                    sim_us = int(round(header_sec * 1e6 + (header_nsec or 0.0) / 1e3))
                    recv_wall = time.time_ns() // 1000
                    recv_mono = time.monotonic_ns() // 1000
                    vx = vy = vz = float("nan")
                    dt = float("nan")
                    if last is not None and sim_us > last[0]:
                        dt = (sim_us - last[0]) * 1e-6
                        if 1e-4 < dt < 0.5:
                            vx = (p["x"] - last[1]) / dt
                            vy = (p["y"] - last[2]) / dt
                            vz = (p["z"] - last[3]) / dt
                    if sim_us > 0 and (last is None or sim_us > last[0]):
                        last = (sim_us, p["x"], p["y"], p["z"])
                    wr.writerow([recv_wall, recv_mono, sim_us, name,
                                 "%.9f" % p["x"], "%.9f" % p["y"], "%.9f" % p["z"],
                                 "%.9f" % q["x"], "%.9f" % q["y"],
                                 "%.9f" % q["z"], "%.9f" % q["w"],
                                 "%.6f" % vx, "%.6f" % vy, "%.6f" % vz,
                                 "%.6f" % dt])
                    rows += 1
                    if rows % 200 == 0:
                        fh.flush()
                        print("truth rows=%d sim=%.3f p=(%.2f,%.2f,%.2f)"
                              % (rows, sim_us * 1e-6, p["x"], p["y"], p["z"]),
                              flush=True)
                else:
                    block.append(s)
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            proc.kill()
        fh.flush()
        fh.close()
        print("truth recorder exit: rows=%d" % rows, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
