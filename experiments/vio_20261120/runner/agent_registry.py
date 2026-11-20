#!/usr/bin/env python3
"""MicroXRCEAgent 登记与按身份回收。

背景：`setsid "$AGENT_BIN" ... &` 让 agent 自成会话（sid == 自身 pid），
因此「sid == 本轮运行器 sid」的回收条件永远不成立，agent 每轮都会残留并占住 8888。
修复方式：启动时立刻登记（pid + /proc/<pid>/stat 第 22 字段 starttime + cmdline），
清理时**只**按登记身份核验，不信 pgrep、不按名字匹配、不用 pkill。

子命令：
  record  --out FILE --binary PATH [--started-by-us 0|1] [--round-sid N] [--pid P]
  reclaim --registry FILE [--grace 2.0] [--dry-run]
  scan    [--json FILE]

退出码：record/reclaim/scan 正常完成均返回 0；只有用法错误返回 2。
"""
import argparse
import json
import os
import signal
import sys
import time


def _proc_stat_fields(pid):
    try:
        with open("/proc/%d/stat" % pid, "r") as f:
            raw = f.read()
    except OSError:
        return None
    # comm 可能含空格/括号，按最后一个 ')' 切分
    idx = raw.rfind(")")
    if idx < 0:
        return None
    rest = raw[idx + 2:].split()
    # rest[0] 是 state（field 3）；field 22 starttime -> rest[19]
    try:
        return {"state": rest[0], "ppid": int(rest[1]), "sid": int(rest[3]),
                "starttime": int(rest[19])}
    except (IndexError, ValueError):
        return None


def _cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            return f.read().replace(b"\x00", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def _exe(pid):
    try:
        return os.path.realpath("/proc/%d/exe" % pid)
    except OSError:
        return ""


def cmd_record(a):
    rec = {
        "schema": "np01_agent_registry/1",
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "started_by_us": bool(a.started_by_us),
        "round_sid": a.round_sid,
        "binary": a.binary,
        "pid": a.pid,
        "available": False,
    }
    if a.pid and a.pid > 0:
        st = _proc_stat_fields(a.pid)
        if st:
            rec.update({
                "available": True,
                "pid": a.pid,
                "starttime": st["starttime"],
                "sid": st["sid"],
                "ppid": st["ppid"],
                "state": st["state"],
                "cmdline": _cmdline(a.pid),
                "exe_realpath": _exe(a.pid),
            })
    # 记录本轮是否**由我们启动**：只有启动过才允许回收，
    # 复用别的任务/上一次残留的 agent 时不许杀。
    tmp = a.out + ".tmp"
    with open(tmp, "w") as f:
        json.dump(rec, f, ensure_ascii=False, indent=2)
    os.replace(tmp, a.out)
    print(json.dumps(rec, ensure_ascii=False))
    return 0


def cmd_reclaim(a):
    res = {"registry": a.registry, "action": "none", "detail": ""}
    try:
        rec = json.load(open(a.registry))
    except (OSError, ValueError) as e:
        res.update(action="skip_no_registry", detail=str(e))
        print(json.dumps(res, ensure_ascii=False))
        return 0

    if not rec.get("started_by_us"):
        res.update(action="skip_not_started_by_us",
                   detail="agent 由其他任务/更早的轮次启动，不回收")
        print(json.dumps(res, ensure_ascii=False))
        return 0

    pid = int(rec.get("pid") or 0)
    if pid <= 0 or not rec.get("available"):
        res.update(action="skip_unavailable", detail="登记时进程已不存在")
        print(json.dumps(res, ensure_ascii=False))
        return 0

    st = _proc_stat_fields(pid)
    if st is None:
        res.update(action="already_gone", detail="pid %d 已不存在" % pid)
        print(json.dumps(res, ensure_ascii=False))
        return 0
    if st["state"] == "Z":
        # 僵尸态：进程已死，只是父进程尚未 reap。不能算存活，也不能再发信号。
        res.update(action="already_gone", detail="pid %d 处于僵尸态" % pid)
        print(json.dumps(res, ensure_ascii=False))
        return 0

    if st["starttime"] != int(rec.get("starttime", -1)):
        # PID 复用：同一 pid 但启动时刻不同 -> 绝不是我们启动的那个进程
        res.update(action="skip_pid_reused",
                   detail="starttime %s != 登记 %s" % (st["starttime"], rec.get("starttime")))
        print(json.dumps(res, ensure_ascii=False))
        return 0

    want = rec.get("binary") or ""
    got = _cmdline(pid)
    if want and want not in got:
        res.update(action="skip_cmdline_mismatch", detail=got[:200])
        print(json.dumps(res, ensure_ascii=False))
        return 0

    if a.dry_run:
        res.update(action="would_terminate", detail="pid %d starttime %s" % (pid, st["starttime"]))
        print(json.dumps(res, ensure_ascii=False))
        return 0

    try:
        os.kill(pid, signal.SIGTERM)
    except OSError as e:
        res.update(action="kill_failed", detail=str(e))
        print(json.dumps(res, ensure_ascii=False))
        return 0

    deadline = time.time() + a.grace
    while time.time() < deadline:
        st2 = _proc_stat_fields(pid)
        if st2 is None or st2["state"] == "Z":
            break
        time.sleep(0.05)
    still = _proc_stat_fields(pid)
    gone = (still is None or still["state"] == "Z")
    res.update(action="terminated" if gone else "still_alive",
               detail="pid %d starttime %s" % (pid, st["starttime"]))
    print(json.dumps(res, ensure_ascii=False))
    return 0


SCAN_PATTERNS = (
    ("px4", "bin/px4"),
    ("gz", "gz-sim-main"),
    ("node", "stereo_inertial_sim_node"),
    ("pilot", "sim_pilot"),
    ("bt", "pilot_navigation"),
    ("supervisor", "np01_truth_supervisor"),
    ("bridge", "ros_gz_bridge"),
    ("agent", "MicroXRCEAgent"),
)


def cmd_scan(a):
    found = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        cmd = _cmdline(pid)
        if not cmd:
            continue
        for tag, pat in SCAN_PATTERNS:
            if pat in cmd:
                st = _proc_stat_fields(pid) or {}
                found.append({"kind": tag, "pid": pid,
                              "starttime": st.get("starttime"), "sid": st.get("sid"),
                              "cmd": cmd[:180]})
                break
    out = {"schema": "np01_agent_scan/1",
           "scanned_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
           "count": len(found), "found": found}
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
    print(json.dumps(out, ensure_ascii=False))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("record")
    r.add_argument("--out", required=True)
    r.add_argument("--binary", required=True)
    r.add_argument("--pid", type=int, default=0)
    r.add_argument("--started-by-us", type=int, default=0)
    r.add_argument("--round-sid", type=int, default=0)
    r.set_defaults(fn=cmd_record)

    c = sub.add_parser("reclaim")
    c.add_argument("--registry", required=True)
    c.add_argument("--grace", type=float, default=2.0)
    c.add_argument("--dry-run", action="store_true")
    c.set_defaults(fn=cmd_reclaim)

    s = sub.add_parser("scan")
    s.add_argument("--json", default="")
    s.set_defaults(fn=cmd_scan)

    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
