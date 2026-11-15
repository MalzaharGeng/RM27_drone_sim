#!/usr/bin/env python3
"""workpkg_20261115 runtime evidence tools.

Replaces the phase7 broad `pgrep -f stereo_inertial_sim_node` heuristic, which
matched the CALLING shell (its cmdline contained ORB_NODE_BIN) and produced a
bogus /proc/<pid>/exe hash.

Identity is now established by:
  1. descending the process tree from the launcher PID,
  2. requiring /proc/<pid>/exe realpath == the expected binary realpath,
  3. requiring process start time >= run start,
  4. capturing while the process is ALIVE (never after exit).
"""
import collections
import csv
import hashlib
import os
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")

ENV_WHITELIST = (
    "PROFILE", "DISTANCE", "RECORD", "PX4_VIO_CONFIG", "ORB_PROFILE",
    "SIM_DDS_DEDUP", "KEEP_RUNS", "WS", "ORB_ROOT", "ORB_NODE_BIN",
    "SIM_LATENCY_OBSERVATION", "SIM_DIAG_EXIT_AFTER", "DURATION",
    "RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "ORB_SETTINGS_FILE",
    "SYNC_TOL", "SIM_NAV_GUARD", "SIM_RUN",
    # [NP-01 FIX16] supervision / handoff evidence
    "NP01_SUPERVISOR", "NP01_TRUTH_WATCHDOG", "NP01_WORLD",
    "SUP_WARN_CLEARANCE", "SUP_STOP_CLEARANCE", "SUP_ENVELOPE",
    "SUP_MAX_XY", "SUP_MAX_XY_INIT", "SUP_INIT_SECONDS", "SUP_ACK_TIMEOUT",
    "INIT_SETTLE", "INIT_GATE", "INIT_GATE_AIRBORNE_HOLD",
    "EXCITE_CONTROLLER", "EXCITE_WAVEFORM", "EXCITE_SECONDS",
    "EXCITE_SLOW_DEG", "EXCITE_SLOW_PERIOD", "EXCITE_FAST_DEG",
    "EXCITE_FAST_PERIOD", "POSITION_HOLD", "TAKEOFF_ALT",
)
KEY_SO = ("libORB_SLAM3", "librclcpp", "libpx4_msgs", "librmw",
          "libDBoW2", "libg2o")


def md5_file(path, chunk=1 << 20):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for c in iter(lambda: f.read(chunk), b""):
            h.update(c)
    return h.hexdigest()


def _stat_tail(pid):
    with open("/proc/%d/stat" % pid, "r") as f:
        line = f.read().strip()
    return line[line.rfind(")") + 2:].split()


def proc_table():
    out = {}
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            t = _stat_tail(pid)
            # t[0]=state, t[1]=ppid, t[19]=starttime (field 22)
            out[pid] = (int(t[1]), int(t[19]), t[0])
        except (OSError, ValueError, IndexError):
            continue
    return out


def descendants(root_pid):
    tab = proc_table()
    kids = {}
    for pid, (ppid, st, state) in tab.items():
        kids.setdefault(ppid, []).append(pid)
    seen, stack, res = set(), [root_pid], []
    while stack:
        p = stack.pop()
        if p in seen:
            continue
        seen.add(p)
        res.append(p)
        stack.extend(kids.get(p, []))
    return set(res), tab


def boot_jiffies():
    with open("/proc/uptime", "r") as f:
        return int(float(f.read().split()[0]) * CLK_TCK)


def find_node_pid(root_pid, expected_exe, min_start=0):
    """Exact identity. A caller shell can never match: we require the
    executable of the CANDIDATE process itself, not its cmdline text."""
    want = os.path.realpath(expected_exe)
    tree, tab = descendants(root_pid)
    hits = []
    for pid in sorted(tree):
        try:
            exe = os.readlink("/proc/%d/exe" % pid)
        except OSError:
            continue
        if exe.endswith(" (deleted)"):
            continue
        if os.path.realpath(exe) != want:
            continue
        st = tab.get(pid, (0, 0, ""))[1]
        if st < min_start:
            continue
        hits.append(pid)
    return hits


def capture_live(pid, expected_md5=None):
    """Capture while alive. Never dumps full environ (no credentials)."""
    d = {"capture_kind": "RUNTIME_LIVE",
         "capture_wall_us": int(time.time() * 1e6),
         "pid": pid,
         "alive_at_capture": os.path.exists("/proc/%d" % pid)}
    try:
        d["exe"] = os.readlink("/proc/%d/exe" % pid)
        d["exe_realpath"] = os.path.realpath(d["exe"])
    except OSError as e:
        d["exe"] = "ERR:%s" % e
    try:
        d["exe_md5"] = md5_file("/proc/%d/exe" % pid)
    except OSError as e:
        d["exe_md5"] = "ERR:%s" % e
    if expected_md5:
        d["exe_md5_expected"] = expected_md5
        d["exe_md5_match"] = (d["exe_md5"] == expected_md5)
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            d["cmdline"] = f.read().replace(b"\0", b" ").decode(errors="replace")[:4000]
    except OSError as e:
        d["cmdline"] = "ERR:%s" % e
    env, hidden = {}, 0
    try:
        with open("/proc/%d/environ" % pid, "rb") as f:
            for kv in f.read().split(b"\0"):
                if b"=" not in kv:
                    continue
                k, v = kv.split(b"=", 1)
                key = k.decode(errors="replace")
                if key in ENV_WHITELIST:
                    env[key] = v.decode(errors="replace")
                else:
                    hidden += 1
    except OSError:
        pass
    d["env_whitelisted"] = env
    d["env_other_keys_not_dumped"] = hidden
    libs = set()
    try:
        with open("/proc/%d/maps" % pid, "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) < 6:
                    continue
                p = parts[5]
                if p.endswith(".so") or ".so." in p:
                    libs.add(p)
    except OSError:
        pass
    d["loaded_so_count"] = len(libs)
    d["loaded_so_key"] = sorted(x for x in libs if any(k in x for k in KEY_SO))
    # [NP-01 FIX5][OBSERVATION ONLY] The algorithm fix lives in the shared
    # library, not in the node executable (the node binary is byte-identical
    # to FIX4-observe). Record a bounded md5 of the ACTUALLY MAPPED
    # libORB_SLAM3.so so the candidate can be proven loaded at runtime.
    orb_so = sorted(x for x in libs if x.endswith("/libORB_SLAM3.so"))
    d["loaded_orbslam3_so"] = orb_so
    if len(orb_so) == 1:
        try:
            d["loaded_orbslam3_so_md5"] = md5_file(orb_so[0])
        except OSError as e:
            d["loaded_orbslam3_so_md5"] = "ERR:%s" % e
    else:
        d["loaded_orbslam3_so_md5"] = "AMBIGUOUS:%d" % len(orb_so)
    try:
        t = _stat_tail(pid)
        d["starttime_jiffies"] = int(t[19])
        d["starttime_s_after_boot"] = round(int(t[19]) / CLK_TCK, 3)
    except (OSError, ValueError, IndexError):
        pass
    return d


def bridge_audit(path):
    """Distinguish MISSING / EMPTY_FILE / PARSE_ERROR / ZERO_DATA_ROWS /
    REAL_ZERO_PUBLISH / PUBLISH_PRESENT. Never collapses these into '0'."""
    r = {"path": str(path)}
    if not os.path.exists(path):
        r["state"] = "MISSING"
        return r
    r["size_bytes"] = os.path.getsize(path)
    if r["size_bytes"] == 0:
        r["state"] = "EMPTY_FILE"
        return r
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
    except Exception as e:
        r["state"] = "PARSE_ERROR"
        r["error"] = str(e)
        return r
    if not rows:
        r["state"] = "ZERO_DATA_ROWS"
        return r
    r["state"] = "OK"
    r["rows"] = len(rows)
    r["cols"] = len(rows[0])
    r["event_counts"] = dict(collections.Counter(x.get("event", "") for x in rows))
    pubs = [x for x in rows if x.get("event") == "publish"]
    r["publish_rows"] = len(pubs)
    r["publish_verdict"] = "PUBLISH_PRESENT" if pubs else "REAL_ZERO_PUBLISH"
    sent = [int(x["sent"]) for x in rows if str(x.get("sent", "")).isdigit()]
    r["sent_max"] = max(sent, default=0)
    r["reason_counts"] = dict(
        collections.Counter(x.get("reason", "") for x in rows).most_common(10))
    return r


def _pct(v, p):
    if not v:
        return None
    s = sorted(v)
    return round(s[min(len(s) - 1, int(len(s) * p))], 2)


def rtf_segments(truth_path, window_s=10.0):
    if not os.path.exists(truth_path):
        return []
    try:
        with open(truth_path, newline="") as f:
            rows = list(csv.DictReader(f))
    except Exception:
        return []
    pts = []
    for r in rows:
        try:
            pts.append((int(r["recv_wall_us"]), int(r["sim_us"])))
        except (KeyError, ValueError, TypeError):
            continue
    if len(pts) < 2:
        return []
    t_start = pts[0][0]
    segs, cur, win_start = [], [], pts[0][0]
    for w, s in pts:
        if w - win_start > window_s * 1e6 and len(cur) >= 2:
            dw = cur[-1][0] - cur[0][0]
            ds = cur[-1][1] - cur[0][1]
            segs.append({"t_rel_s": round((cur[0][0] - t_start) / 1e6, 1),
                         "rtf": round(ds / dw, 5) if dw else None,
                         "rows": len(cur)})
            cur, win_start = [], w
        cur.append((w, s))
    if len(cur) >= 2:
        dw = cur[-1][0] - cur[0][0]
        ds = cur[-1][1] - cur[0][1]
        segs.append({"t_rel_s": round((cur[0][0] - t_start) / 1e6, 1),
                     "rtf": round(ds / dw, 5) if dw else None,
                     "rows": len(cur)})
    return segs


def timing_evidence(latency_path, slam_log):
    out = {}
    if os.path.exists(latency_path):
        try:
            with open(latency_path, newline="") as f:
                rows = list(csv.DictReader(f))
            out["latency_rows"] = len(rows)
            rx, track, proc, resid = [], [], [], []
            for r in rows:
                try:
                    rx.append(int(r["rx_wall_us"]))
                    track.append((int(r["track_end_us"]) - int(r["track_start_us"])) / 1000.0)
                    proc.append((int(r["submit_wall_us"]) - int(r["rx_wall_us"])) / 1000.0)
                    resid.append(abs((int(r["sample_us"]) - int(r["src_sim_us"]))
                                     - int(r["map_offset_us"])))
                except (KeyError, ValueError, TypeError):
                    continue
            gaps = [(rx[i] - rx[i - 1]) / 1000.0 for i in range(1, len(rx))]
            out["rx_gap_ms"] = {"p50": _pct(gaps, .5), "p95": _pct(gaps, .95),
                                "max": round(max(gaps), 2) if gaps else None}
            out["track_ms"] = {"p50": _pct(track, .5), "p95": _pct(track, .95)}
            out["rx_to_submit_ms"] = {"p50": _pct(proc, .5), "p95": _pct(proc, .95)}
            out["sample_minus_src_eq_offset_max_residual_us"] = max(resid, default=None)
        except Exception as e:
            out["latency_error"] = str(e)
    else:
        out["latency_state"] = "MISSING"
    if os.path.exists(slam_log):
        try:
            txt = open(slam_log, errors="replace").read().splitlines()
            out["clock_reject_warn_lines"] = sum(1 for l in txt if "clock reject" in l)
            out["epoch_transition_lines"] = sum(1 for l in txt if "sensor clock epoch ->" in l)
            out["first_clock_reject"] = next(
                (l.strip() for l in txt if "clock reject" in l), None)
        except OSError:
            pass
    return out
