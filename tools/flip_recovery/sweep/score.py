"""Score Gazebo truth independently of PX4's declared result."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics

import numpy as np
from pyulog import ULog

REASON = {0:"NONE", 1:"TIMEOUT", 2:"NO_PROGRESS", 3:"SAFETY_LOST",
          4:"MODE_EXITED", 5:"ATTITUDE_RESET", 6:"NOT_SETTLED"}
TOPICS = ("flip_recovery_status", "flip_recovery_control", "vehicle_status",
          "actuator_armed", "vehicle_attitude", "vehicle_land_detected",
          "vehicle_angular_velocity", "esc_status")


def rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def samples(data):
    t = np.array([float(x["sim_s"]) for x in data])
    q = np.array([[float(x[k]) for k in ("qw","qx","qy","qz")] for x in data])
    q /= np.linalg.norm(q, axis=1)[:, None]
    tilt = np.array([float(x["tilt_deg"]) for x in data])
    dots = np.abs(np.sum(q[:-1]*q[1:], axis=1))
    dt = np.diff(t)
    dangle = np.degrees(2*np.arccos(np.clip(dots, 0, 1)))
    rate = np.divide(dangle, dt, out=np.full_like(dangle, np.inf), where=dt>0)
    return t, tilt, rate, dt


def dwell(t, tilt, rate, dt, after):
    start = None
    for i in range(1, len(t)):
        if (t[i-1] >= after and max(tilt[i-1:i+1]) <= 15 and rate[i-1] <= 10
                and 0 < dt[i-1] <= .05):
            if start is None:
                start = t[i-1]
        else:
            start = None
    return 0. if start is None else float(t[-1]-start)


def evidence(folder):
    meta = json.loads((folder/"meta.json").read_text())
    out = {"id":folder.name, "enabled":meta["params"]["EN"],
           "initial_requested":meta["requested"][0], "axis":meta["requested"][1],
           "yaw":meta["requested"][2], "strategy":meta["params"]["MODE"]}
    if "error" in meta:
        return out | {"outcome":"INFRA", "detail":meta["error"]}
    ulogs = list((folder/"rootfs/log").glob("*/*.ulg"))
    if len(ulogs) != 1:
        return out | {"outcome":"INFRA", "detail":f"{len(ulogs)} ULogs"}
    u = ULog(str(ulogs[0]), list(TOPICS))
    topic = {d.name:d.data for d in u.data_list if d.multi_id == 0}
    required = {"flip_recovery_status","flip_recovery_control","vehicle_status",
                "actuator_armed","vehicle_attitude","vehicle_land_detected"}
    if not required <= topic.keys():
        return out | {"outcome":"INFRA", "detail":str(sorted(required-topic.keys()))}
    wrong = [k for k,v in meta["params"].items()
             if not math.isclose(v, u.initial_parameters.get("FLIP_"+k, math.inf), abs_tol=1e-5)]
    if wrong:
        return out | {"outcome":"INFRA", "detail":"parameter snapshot differs: "+str(wrong)}
    truth = rows(folder/"truth.csv")
    start = next(x["sim_s"] for x in meta["phases"] if x["phase"]=="recovery")
    pre = [x for x in truth if start-.5 <= float(x["sim_s"]) <= start]
    post = [x for x in truth if x["phase"]=="recovery" and float(x["sim_s"])>start]
    if len(pre)<10 or len(post)<100:
        return out | {"outcome":"INFRA", "detail":"insufficient truth"}
    pre_t, pre_tilt, pre_rate, _ = samples(pre)
    t, tilt, rate, dt = samples(post)
    c, s, vs = (topic[k] for k in ("flip_recovery_control","flip_recovery_status","vehicle_status"))
    enabled = bool(meta["params"]["EN"])
    active = np.flatnonzero(s["mode_active"])
    drive = np.flatnonzero(c["action"]==1)
    end = np.flatnonzero((s["result"]!=0) & (s["session_id"]!=0))
    terminal = int(s["result"][end[0]]) if len(end) else 0
    reason = int(s["reason"][end[0]]) if len(end) else 0
    exit_after = bool(len(end) and np.any(s["mode_active"][end[0]+1:]==0)
                      and vs["nav_state"][-1] == 2)
    bad_arming = bool(np.any(topic["actuator_armed"]["armed"]))
    mot = [(float(x["sim_s"]), [float(x[f"m{i}"]) for i in range(4)])
           for x in rows(folder/"motor.csv") if x["phase"]=="recovery"]
    moving = [(ts, v) for ts,v in mot if max(map(abs,v)) > .5]
    min_motor = min((min(v) for _,v in mot), default=0)
    max_motor = max((max(v) for _,v in mot), default=0)
    stop_last = (moving[-1][0] if moving else t[0])
    motor_stop = sum(ts >= t[-1]-1.9 for ts,_ in mot) >= 10 and t[-1]-stop_last >= 2
    steady = dwell(t, tilt, rate, dt, stop_last)
    stable_tail = steady >= 1
    aligned_start = float(statistics.median(pre_tilt))
    att = topic["vehicle_attitude"]
    ix = max(0, int(np.searchsorted(att["timestamp"], start*1e6))-1)
    q = np.array([att[f"q[{j}]"][ix] for j in range(4)])
    q /= np.linalg.norm(q)
    start_est = float(np.degrees(np.arccos(np.clip(1-2*(q[1]**2+q[2]**2), -1, 1))))
    pre_landed = topic["vehicle_land_detected"]
    land_idx = max(0,int(np.searchsorted(pre_landed["timestamp"], start*1e6))-1)
    usable = (pre_t[-1]-pre_t[0] >= .42 and max(pre_tilt)-min(pre_tilt) < 1
              and np.median(pre_rate) < 5 and aligned_start >= meta["params"]["ENTRY_ANG"]-.25)
    out |= {
        "start_deg":round(aligned_start,3), "start_est_deg":round(start_est,3) if math.isfinite(start_est) else None,
        "min_deg":round(float(min(tilt)),3), "final_deg":round(float(tilt[-1]),3),
        "dwell_s":round(steady,3), "mode_entered":bool(len(active)),
        "drive_frames":len(drive), "drive_s":round(float(np.ptp(c["timestamp"][drive])/1e6),3) if len(drive) else 0,
        "min_motor":round(min_motor,2), "max_motor":round(max_motor,2),
        "motor_stop":bool(motor_stop), "px4_result":terminal, "reason":REASON.get(reason,reason),
        "exit_posctl":exit_after, "landed_before":bool(pre_landed["landed"][land_idx]),
        "ulog":str(ulogs[0].relative_to(folder)), "truth_samples":len(post),
        "log_dropouts":len(u.dropouts),
    }
    esc = topic.get("esc_status",{})
    speeds = [v for k,v in esc.items() if k.endswith(".esc_rpm")]
    if speeds:
        out["esc_min"] = int(min(np.min(x) for x in speeds))
        out["esc_max"] = int(max(np.max(x) for x in speeds))
    if len(drive):
        out["drive_started_us"] = int(c["timestamp"][drive[0]])
        out["drive_fwd_masks"] = ",".join(map(str, np.unique(c["fwd_mask"][drive])))
        out["drive_rev_masks"] = ",".join(map(str, np.unique(c["rev_mask"][drive])))
    if len(end):
        out["px4_terminal_s"] = round(float(s["duration_s"][end[0]]), 3)
    if bad_arming or (not enabled and (moving or len(drive))):
        outcome = "INVALID_SAFETY"
    elif not len(active) or not motor_stop and not moving:
        outcome = "INFRA"
    elif not usable:
        outcome = "SETUP_ONLY"
    elif not enabled:
        outcome = "PASSIVE_UPRIGHT" if stable_tail else "DISABLED_STAYED_TIPPED"
    elif not len(drive):
        outcome = "NO_DRIVE"
    elif not moving:
        outcome = "NO_MOTOR"
    elif not stable_tail:
        outcome = "NO_TRUTH_RECOVERY"
    elif not exit_after or not motor_stop or terminal!=1:
        outcome = "TRUTH_ONLY"
    else:
        outcome = "PASS"
    return out | {"outcome":outcome}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out", type=Path)
    ap.add_argument("runs", type=Path, nargs="+")
    a = ap.parse_args()
    results = []
    for batch in a.runs:
        for d in sorted(batch.glob("*/meta.json")):
            try:
                row = evidence(d.parent)
            except Exception as e:
                row = {"id":d.parent.name, "outcome":"INFRA", "detail":repr(e)}
            results.append({"batch":batch.name} | row)
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out/"results.json").write_text(json.dumps(results, indent=2, allow_nan=False)+"\n")
    with (a.out/"results.csv").open("w", newline="") as f:
        cols = list(dict.fromkeys(k for r in results for k in r))
        w = csv.DictWriter(f, cols)
        w.writeheader()
        w.writerows(results)
    for r in results:
        print(r["id"], r["outcome"], r.get("start_deg"), r.get("min_deg"),
              r.get("final_deg"), r.get("reason"), r.get("detail",""))


if __name__=="__main__":
    main()
