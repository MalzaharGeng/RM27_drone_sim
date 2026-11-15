#!/usr/bin/env python3
"""为单轮实验写 manifest.json（可复现所需的全部实查信息）。

只读取本机事实，不做任何推断：
  * 运行目录、profile、指令距离、VIO 门控配置；
  * 运行二进制/库的路径、md5、大小、mtime；
  * ORB_SLAM3 / drone_sim / rm27_slam 的 git HEAD 与脏文件清单；
  * 相机标定、门控 yaml、行为树 config 的 md5；
  * 环境开关快照（重点是确认没有 PILOT_LINK_PROBE / WSL_RELAX_FRESHNESS）；
  * 起止时间与宿主机信息。

用法：
  write_manifest.py --run <dir> --profile control_truth|vio_closed_loop \
      --distance 5.0 --t0 <epoch> --t1 <epoch> --vio-config strict
"""
import argparse
import hashlib
import json
import os
import platform
import socket
import subprocess
import time
from pathlib import Path

# 参与链路的二进制/库。缺失就记为 null，绝不猜。
FILES = {
    "px4_bin": "/home/gengyinghao/drone_sim/PX4-Autopilot/build/px4_sitl_default/bin/px4",
    "orb_node": "/home/gengyinghao/px4_ws_201/install/rm27_slam/lib/rm27_slam/stereo_inertial_sim_node",
    "truth_odom_node": "/home/gengyinghao/px4_ws_201/install/rm27_slam/lib/rm27_slam/truth_odom_node",
    "liborb": "/home/gengyinghao/ORB_SLAM3/build/lib/libORB_SLAM3.so",
    "px4_core": "/home/gengyinghao/ORB_SLAM3/build/liborbslam3_px4_core.a",
    "settings_yaml": "/home/gengyinghao/ORB_SLAM3/config/RM27_Sim_IMU_Stereo.yaml",
    "px4_vio_strict": "/home/gengyinghao/ORB_SLAM3/config/px4_vio_sim.yaml",
    "px4_vio_transport": "/home/gengyinghao/ORB_SLAM3/config/px4_vio_sim_transport.yaml",
    "bt_config_base": "/home/gengyinghao/drone_sim/config/pilot_navigation_link_5m.json",
}

REPOS = {
    "ORB_SLAM3": "/home/gengyinghao/ORB_SLAM3",
    "drone_sim": "/home/gengyinghao/drone_sim",
    "rm27_slam": "/home/gengyinghao/px4_ws_201/src/rm27_slam",
}

# 这些变量一旦存在，本轮结论不能作为「真实链路」验收。
FORBIDDEN_ENV = ("PILOT_LINK_PROBE", "WSL_RELAX_FRESHNESS")
# 这些变量影响链路语义，必须原样记录。
TRACKED_ENV = ("PROFILE", "DISTANCE", "PX4_VIO_CONFIG", "SYNC_TOL", "RECORD",
               "SIM_NAV_GUARD", "WS", "ORB_ROOT", "ROS_DISTRO", "RMW_IMPLEMENTATION",
               "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE",
               "TRUTH_ENTITY", "SIM_DDS_DEDUP", "ORB_PROFILE",
               "ORB_SETTINGS_FILE", "GZ_SIM_RESOURCE_PATH",
               # [NP-01 FIX12] audit-only: prove which candidate path ran.
               # These are NOT forbidden switches and affect no verdict.
               "EXCITE_CONTROLLER", "NP01_GATE_SNAPSHOT",
               "NP01_TRUTH_WATCHDOG",
               # [NP-01 FIX13] audit-only: init diagnostics + bounded wait cap.
               # Read-only observation; no Gate/LIMITS/age/verdict change.
               "ORB_INIT_METRICS", "EXCITE_WAIT_CAP_S")


def md5(path):
    try:
        h = hashlib.md5()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def file_info(path):
    p = Path(path)
    try:
        st = p.stat()
    except OSError:
        return {"path": path, "exists": False}
    return {
        "path": path,
        "exists": True,
        "size": st.st_size,
        "mtime": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(st.st_mtime)),
        "mtime_epoch": round(st.st_mtime, 3),
        "md5": md5(path),
    }


def run(cmd, cwd):
    try:
        out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=20)
        return out.stdout.strip()
    except Exception:
        return ""


def git_info(path):
    if not Path(path, ".git").exists() and not run(["git", "rev-parse", "--git-dir"], path):
        return {"path": path, "is_repo": False}
    head = run(["git", "rev-parse", "HEAD"], path)
    branch = run(["git", "rev-parse", "--abbrev-ref", "HEAD"], path)
    status = run(["git", "status", "--porcelain"], path)
    lines = [l for l in status.splitlines() if l.strip()]
    return {
        "path": path,
        "is_repo": True,
        "head": head or None,
        "branch": branch or None,
        "dirty": bool(lines),
        "dirty_count": len(lines),
        "dirty_files": lines[:60],
    }


def env_snapshot():
    snap = {}
    for k in TRACKED_ENV:
        snap[k] = os.environ.get(k)
    forbidden = {}
    for k in FORBIDDEN_ENV:
        v = os.environ.get(k)
        forbidden[k] = {"present": k in os.environ, "value": v}
    return snap, forbidden


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--profile", required=True)
    ap.add_argument("--distance", type=float, required=True)
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=0.0)
    ap.add_argument("--vio-config", default="strict")
    args = ap.parse_args()

    run_dir = Path(args.run)
    env, forbidden = env_snapshot()
    files = {k: file_info(v) for k, v in FILES.items()}
    bt_cfg = run_dir / "bt_config.json"
    if bt_cfg.exists():
        files["bt_config_run"] = file_info(str(bt_cfg))
    settings = os.environ.get("ORB_SETTINGS_FILE")
    if settings:
        files["orb_settings_run"] = file_info(settings)

    manifest = {
        "schema_version": 1,
        "run_dir": str(run_dir),
        "profile": args.profile,
        "distance_m": args.distance,
        "vio_config": args.vio_config,
        # transport 配置只改 timesync / 样本年龄两项，属 [SIM-ONLY] 传输补偿。
        "transport_compensation": args.vio_config == "transport",
        "started_epoch": args.t0 or None,
        "finished_epoch": args.t1 or None,
        "started_local": (time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(args.t0))
                          if args.t0 else None),
        "finished_local": (time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(args.t1))
                           if args.t1 else None),
        "wall_duration_s": round(args.t1 - args.t0, 3) if args.t0 and args.t1 else None,
        "host": {
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "kernel": platform.release(),
            "python": platform.python_version(),
            "cpu_count": os.cpu_count(),
            "ros_distro": os.environ.get("ROS_DISTRO"),
        },
        "env": env,
        "forbidden_env": forbidden,
        "forbidden_env_present": [k for k, v in forbidden.items() if v["present"]],
        "files": files,
        "repos": {k: git_info(v) for k, v in REPOS.items()},
        "source_newer_than_binary": {},
    }

    # 源码 vs 运行二进制：只报告 mtime 事实，便于发现「跑的不是刚改的代码」。
    src_newer = manifest["source_newer_than_binary"]
    checks = [
        ("orb_node", ["/home/gengyinghao/px4_ws_201/src/rm27_slam/src/stereo_inertial_sim_node.cpp"]),
        ("orb_node", ["/home/gengyinghao/px4_ws_201/src/rm27_slam/src/truth_odom_node.cpp"]),
        ("liborb", ["/home/gengyinghao/ORB_SLAM3/src/Px4Vio.cc",
                    "/home/gengyinghao/ORB_SLAM3/src/Px4Bridge.cc"]),
    ]
    for bin_key, srcs in checks:
        bin_info = files.get(bin_key) or {}
        bt = bin_info.get("mtime_epoch")
        newer = []
        for s in srcs:
            st = Path(s).stat() if Path(s).exists() else None
            if st and bt and st.st_mtime > bt:
                newer.append({"source": s, "mtime": time.strftime(
                    "%Y-%m-%dT%H:%M:%S", time.localtime(st.st_mtime))})
        src_newer[bin_key + ":" + ",".join(Path(s).name for s in srcs)] = newer

    out = run_dir / "manifest.json"
    out.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print("manifest written:", out)
    if manifest["forbidden_env_present"]:
        print("!! 禁止的绕过变量存在:", manifest["forbidden_env_present"])


if __name__ == "__main__":
    main()
