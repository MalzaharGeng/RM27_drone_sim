#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX18] 单轮进程归属 / 身份 / 清理（证据与清理共用一套实现）。

背景（缺陷）
------------
FIX17 的运行环境里，ros_gz_bridge 是 run_stereo_inertial_sim.sh 自己启动的，
该脚本的 EXIT trap 会 kill 掉这个桥。于是「精确终止唯一 VIO 节点」会连带
杀掉桥，进而让监督器丢失真值位姿输入（第 23 轮实测 1510.8 ms），按
fail-closed 暂停物理，降落无法完成。

本模块把「谁启动、谁拥有、谁回收」变成可核查的数据，而不是一条 kill 命令：

* 归属由两个独立事实共同决定
    1) 本轮登记表里有该 PID，且启动时刻（/proc/<pid>/stat 第 22 字段）
       与登记时一致；
    2) 该进程的会话号 sid == 本轮运行器的 sid。
  两者同时成立才允许发信号。从不按名字或 cmdline 子串匹配进程，
  所以别的任务起的共享进程（例如 setsid 启动的 MicroXRCEAgent）不会被抓到。
* 不使用 pkill。
* 正常结束 / 异常退出 / 注入退出 走同一个 cleanup 入口，只是 reason 不同。
* 清理后立即复核：本轮会话内是否还有非僵尸进程；结果写进 JSON 证据。

子命令
------
  sid        --pid P                     打印 P 的会话号
  add        --run-dir D --role R --pid P  登记一个已知 PID 及其身份
  scan       --run-dir D --sid S         按会话归属扫描并补登记（角色标签）
  watch      --run-dir D [--interval 1]  周期采样登记表的存活/身份一致性
  cleanup    --run-dir D --sid S --reason normal|abnormal|injected [--grace 6]
  verify     --run-dir D --sid S         仅复核，不发信号
  signal-pid --run-dir D --pid P --expect-exe PATH [--signal TERM] --out F
  reclaim    --exe PATH [--world W] [--out F] [--dry-run]
                                         仅回收本人构建/本 world 的历史残留
"""
import argparse
import json
import os
import signal
import sys
import time

PROC = "/proc"
ZOMBIE_STATES = ("Z", "X", "x")

# 清理顺序：先让会写证据/发指令的进程停下来，再断开桥，最后停物理。
ROLE_ORDER = [
    "pilot", "lbt", "supervisor", "telemetry", "obs_counter", "recorder",
    "node", "launcher", "bridge", "px4", "gz", "session_member",
]


def now_us():
    return int(time.time() * 1e6)


def mono_us():
    return time.monotonic_ns() // 1000


def _read_bytes(path):
    try:
        with open(path, "rb") as fh:
            return fh.read()
    except OSError:
        return None


def ident(pid):
    """返回进程身份；进程已消失返回 None。comm 可含空格/括号，按最后一个 ')' 切分。"""
    raw = _read_bytes("%s/%d/stat" % (PROC, pid))
    if raw is None:
        return None
    try:
        s = raw.decode("utf-8", "replace")
        idx = s.rfind(")")
        f = s[idx + 2:].split()
        state = f[0]
        ppid = int(f[1])
        pgrp = int(f[2])
        sid = int(f[3])
        starttime = int(f[19])
    except (IndexError, ValueError):
        return None
    try:
        exe = os.readlink("%s/%d/exe" % (PROC, pid))
    except OSError:
        exe = ""
    cmd = _read_bytes("%s/%d/cmdline" % (PROC, pid)) or b""
    cmdline = cmd.replace(b"\0", b" ").decode("utf-8", "replace").strip()
    return {
        "pid": int(pid), "ppid": ppid, "pgrp": pgrp, "sid": sid,
        "state": state, "starttime_s_boot": starttime,
        "exe": exe, "exe_realpath": os.path.realpath(exe) if exe else "",
        "cmdline": cmdline,
    }


def alive(pid):
    i = ident(pid)
    return bool(i) and i["state"] not in ZOMBIE_STATES


def session_members(sid):
    out = {}
    for name in os.listdir(PROC):
        if not name.isdigit():
            continue
        i = ident(int(name))
        if i and i["sid"] == sid:
            out[i["pid"]] = i
    return out


def self_and_ancestors():
    """清理进程自身与其祖先链：运行器必须活着走完自己的收尾。"""
    keep = set()
    pid = os.getpid()
    for _ in range(64):
        keep.add(pid)
        i = ident(pid)
        if not i or i["ppid"] <= 1:
            break
        pid = i["ppid"]
    return keep


def classify(i):
    """给会话内进程打角色标签（只用于顺序与证据，不决定是否可杀）。"""
    exe = i.get("exe_realpath") or ""
    cmd = i.get("cmdline") or ""
    if exe.endswith("/bin/px4"):
        return "px4"
    if "gz-sim-main" in cmd or "gz-sim-server" in cmd or "gz sim" in cmd.split()[:2]:
        return "gz"
    if "parameter_bridge" in cmd and "ros_gz_bridge" in cmd:
        return "bridge"
    if "stereo_inertial_sim_fix18.sh" in cmd or "stereo_inertial_sim.sh" in cmd:
        return "launcher"
    if exe.endswith("/stereo_inertial_sim_node") or "stereo_inertial_sim_node" in os.path.basename(exe):
        return "node"
    if "pilot_navigation_bt.py" in cmd:
        return "lbt"
    if "pilot_navigation_boot.py" in cmd or "sim_pilot.py" in cmd:
        return "pilot"
    if "np01_truth_supervisor.py" in cmd:
        return "supervisor"
    if "vio_obs_counter.py" in cmd:
        return "obs_counter"
    if "telemetry.py" in cmd:
        return "telemetry"
    if "truth_recorder.py" in cmd or "truth_gz_node.py" in cmd or "truth_odom_node" in cmd:
        return "recorder"
    return "session_member"


def registry_path(run_dir):
    return os.path.join(run_dir, "proc_registry.jsonl")


def load_registry(run_dir):
    """返回 {pid: entry}；同一 PID 多次登记时合并 roles。"""
    reg = {}
    path = registry_path(run_dir)
    if not os.path.exists(path):
        return reg
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                e = json.loads(line)
            except ValueError:
                continue
            pid = int(e["pid"])
            if pid in reg:
                roles = reg[pid].setdefault("roles", [reg[pid].get("role")])
                if e.get("role") and e["role"] not in roles:
                    roles.append(e["role"])
                continue
            e["roles"] = [e.get("role")]
            reg[pid] = e
    return reg


def add_entry(run_dir, role, pid, note=""):
    i = ident(pid)
    rec = {"role": role, "pid": int(pid), "recorded_wall_us": now_us(),
           "recorded_mono_us": mono_us(), "note": note}
    if i:
        rec.update({"starttime_s_boot": i["starttime_s_boot"], "sid": i["sid"],
                    "exe_realpath": i["exe_realpath"], "cmdline": i["cmdline"][:400]})
    else:
        rec.update({"missing": True})
    os.makedirs(run_dir, exist_ok=True)
    with open(registry_path(run_dir), "a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
    return rec


def write_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, ensure_ascii=False, indent=2, sort_keys=True)
        fh.write("\n")
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def identity_ok(entry, cur):
    """登记身份与当前身份一致（防止 PID 复用误杀）。"""
    if cur is None:
        return False
    if "starttime_s_boot" not in entry:
        return False
    return int(entry["starttime_s_boot"]) == int(cur["starttime_s_boot"])


def cmd_sid(a):
    i = ident(a.pid)
    if not i:
        return 2
    print(i["sid"])
    return 0


def cmd_add(a):
    rec = add_entry(a.run_dir, a.role, a.pid, a.note or "")
    print(json.dumps(rec, ensure_ascii=False))
    return 0 if not rec.get("missing") else 3


def cmd_scan(a):
    mem = session_members(a.sid)
    reg = load_registry(a.run_dir)
    added = []
    for pid, i in sorted(mem.items()):
        if pid in reg or pid in self_and_ancestors():
            continue
        role = classify(i)
        add_entry(a.run_dir, role, pid, note="scan:session_member")
        added.append({"pid": pid, "role": role, "exe": i["exe_realpath"],
                      "cmdline": i["cmdline"][:200]})
    print(json.dumps({"sid": a.sid, "session_members": len(mem), "added": added},
                     ensure_ascii=False, indent=2))
    return 0


def cmd_watch(a):
    path = os.path.join(a.run_dir, "proc_watch.csv")
    new = not os.path.exists(path)
    fh = open(path, "a", encoding="utf-8")
    if new:
        fh.write("t_mono_s,t_wall_us,role,pid,alive,state,starttime_match,exe\n")
        fh.flush()
    t0 = mono_us()
    try:
        while True:
            reg = load_registry(a.run_dir)
            for pid, e in sorted(reg.items()):
                cur = ident(pid)
                role = e.get("role") or "?"
                fh.write("%.3f,%d,%s,%d,%d,%s,%d,%s\n" % (
                    (mono_us() - t0) * 1e-6, now_us(), role, pid,
                    1 if (cur and cur["state"] not in ZOMBIE_STATES) else 0,
                    (cur or {}).get("state", "-"),
                    1 if identity_ok(e, cur) else 0,
                    (cur or {}).get("exe_realpath", "-")))
            fh.flush()
            time.sleep(a.interval)
    except KeyboardInterrupt:
        pass
    finally:
        fh.flush()
        fh.close()
    return 0


def _ordered_targets(run_dir, sid, exclude):
    reg = load_registry(run_dir)
    mem = session_members(sid)
    targets = []
    seen = set()
    for pid, i in mem.items():
        if pid in exclude:
            continue
        e = reg.get(pid)
        entry = e or {"pid": pid, "role": classify(i), "roles": [classify(i)]}
        if e and identity_ok(e, i):
            identity = "MATCH"
        elif e:
            identity = "MISMATCH"
        else:
            identity = "SESSION_ONLY"
        role = entry.get("role") or classify(i)
        targets.append({
            "pid": pid, "role": role,
            "roles": entry.get("roles") or [role],
            "identity": identity,
            "exe": i["exe_realpath"], "cmdline": i["cmdline"][:300],
            "starttime_s_boot": i["starttime_s_boot"], "state": i["state"],
        })
        seen.add(pid)
    # 登记表里已消失的 PID 也记录，便于区分「本来就没起来」与「被杀掉」
    for pid, e in reg.items():
        if pid in seen or pid in exclude:
            continue
        targets.append({
            "pid": pid, "role": e.get("role") or "?", "roles": e.get("roles") or [],
            "identity": "GONE_AT_CLEANUP",
            "exe": e.get("exe_realpath", ""), "cmdline": (e.get("cmdline") or "")[:300],
            "starttime_s_boot": e.get("starttime_s_boot"), "state": "gone",
        })

    def key(t):
        r = t["role"] if t["role"] in ROLE_ORDER else "session_member"
        return (ROLE_ORDER.index(r), t["pid"])

    targets.sort(key=key)
    return targets


def _signal_verified(pid, sig, expect_starttime=None):
    """发信号前再读一次身份；启动时刻不一致直接拒绝（防 PID 复用）。"""
    cur = ident(pid)
    if cur is None:
        return {"pid": pid, "sent": None, "reason": "gone"}
    if cur["state"] in ZOMBIE_STATES:
        return {"pid": pid, "sent": None, "reason": "zombie"}
    if expect_starttime is not None and int(cur["starttime_s_boot"]) != int(expect_starttime):
        return {"pid": pid, "sent": None, "reason": "starttime_mismatch",
                "now": cur["starttime_s_boot"], "expect": expect_starttime}
    try:
        os.kill(pid, sig)
    except OSError as exc:
        return {"pid": pid, "sent": None, "reason": "oserror:%r" % (exc,)}
    return {"pid": pid, "sent": sig}


def _wait_gone(pids, timeout_s):
    t0 = time.time()
    pending = set(pids)
    while pending and (time.time() - t0) < timeout_s:
        pending = {p for p in pending if alive(p)}
        if pending:
            time.sleep(0.1)
    return sorted(pending)


def cmd_cleanup(a):
    t_start = now_us()
    exclude = self_and_ancestors()
    out = os.path.join(a.run_dir, "process_cleanup.json")

    pre = _ordered_targets(a.run_dir, a.sid, exclude)
    write_json(os.path.join(a.run_dir, "process_snapshot_pre_cleanup.json"),
               {"reason": a.reason, "sid": a.sid, "written_wall_us": t_start,
                "targets": pre})

    for t in pre:
        if t["identity"] == "MISMATCH":
            t["signal"] = "SKIPPED_IDENTITY_MISMATCH"
            t["exited"] = alive(t["pid"]) is False
            continue
        if t["identity"] == "GONE_AT_CLEANUP" or t["state"] in ZOMBIE_STATES:
            t["signal"] = "ALREADY_GONE"
            t["exited"] = True
            continue
        r = _signal_verified(t["pid"], signal.SIGTERM, t.get("starttime_s_boot"))
        t["term"] = r
        t["signal"] = "SIGTERM" if r.get("sent") else "NOOP:%s" % r.get("reason")
        t["t_term_mono_us"] = mono_us()
        time.sleep(0.12)

    pending = _wait_gone([t["pid"] for t in pre
                          if t["signal"] == "SIGTERM"], a.grace)
    for t in pre:
        if t["pid"] in pending:
            r = _signal_verified(t["pid"], signal.SIGKILL, t.get("starttime_s_boot"))
            t["kill"] = r
            t["signal"] += "+SIGKILL" if r.get("sent") else "+KILL_NOOP"
    if pending:
        _wait_gone(pending, 3.0)

    post_members = session_members(a.sid)
    leftovers = []
    for pid, i in post_members.items():
        if pid in exclude:
            continue
        leftovers.append({"pid": pid, "state": i["state"], "exe": i["exe_realpath"],
                          "cmdline": i["cmdline"][:200],
                          "role": classify(i)})

    for t in pre:
        cur = ident(t["pid"])
        t["exited"] = (cur is None) or (cur["state"] in ZOMBIE_STATES)
        t["exit_state"] = (cur or {}).get("state", "gone")
        t["t_done_mono_us"] = mono_us()

    result = {
        "reason": a.reason, "sid": a.sid,
        "started_wall_us": t_start, "finished_wall_us": now_us(),
        "grace_s": a.grace,
        "ownership_rule": ("registry identity (pid + starttime) AND sid == round sid; "
                           "no name/cmdline matching; no pkill"),
        "targets": pre,
        "leftovers": leftovers,
        "verified_empty": len(leftovers) == 0,
        "self_excluded": sorted(exclude),
    }
    write_json(out, result)
    print(json.dumps({"cleanup": out, "targets": len(pre),
                      "leftovers": len(leftovers),
                      "verified_empty": result["verified_empty"]},
                     ensure_ascii=False))
    return 0 if result["verified_empty"] else 5


def cmd_verify(a):
    exclude = self_and_ancestors()
    mem = session_members(a.sid)
    left = [{"pid": p, "state": i["state"], "exe": i["exe_realpath"],
             "cmdline": i["cmdline"][:200]} for p, i in mem.items()
            if p not in exclude and i["state"] not in ZOMBIE_STATES]
    print(json.dumps({"sid": a.sid, "alive_members": left, "clean": not left},
                     ensure_ascii=False, indent=2))
    return 0 if not left else 5


def cmd_signal_pid(a):
    cur = ident(a.pid)
    res = {"requested_wall_us": now_us(), "requested_mono_us": mono_us(),
           "target_pid": a.pid, "expect_exe": a.expect_exe,
           "signal": a.signal}
    if cur is None:
        res.update({"result": "target_gone"})
    else:
        want = os.path.realpath(a.expect_exe) if a.expect_exe else None
        ok_exe = (not want) or (cur["exe_realpath"] == want)
        ok_start = True
        if a.expect_starttime is not None:
            ok_start = int(cur["starttime_s_boot"]) == int(a.expect_starttime)
        res.update({"identity": cur, "exe_match": ok_exe, "starttime_match": ok_start})
        if not (ok_exe and ok_start):
            res.update({"result": "refused_identity_mismatch"})
        else:
            sig = {"TERM": signal.SIGTERM, "INT": signal.SIGINT,
                   "KILL": signal.SIGKILL, "STOP": signal.SIGSTOP}[a.signal]
            try:
                os.kill(a.pid, sig)
                res.update({"result": "sent", "sent_mono_us": mono_us()})
            except OSError as exc:
                res.update({"result": "oserror", "error": repr(exc)})
    if a.out:
        write_json(a.out, res)
    print(json.dumps(res, ensure_ascii=False))
    return 0 if res.get("result") == "sent" else 1


def cmd_reclaim(a):
    """只回收「本人构建的 PX4」与「本人 world 的 gz server」，不碰别的任务。"""
    want_exe = os.path.realpath(a.exe) if a.exe else None
    hits = []
    for name in os.listdir(PROC):
        if not name.isdigit():
            continue
        pid = int(name)
        i = ident(pid)
        if not i or i["state"] in ZOMBIE_STATES:
            continue
        if want_exe and i["exe_realpath"] == want_exe:
            hits.append((pid, i, "px4_exact_exe"))
            continue
        if a.world and ("gz-sim-main" in i["cmdline"] or "gz-sim-server" in i["cmdline"]) \
                and a.world in i["cmdline"]:
            hits.append((pid, i, "gz_exact_world"))
            continue
        # 上一轮崩溃遗留的桥也只按「本 world 的真值话题」识别，不按名字通杀。
        if a.world and "parameter_bridge" in i["cmdline"] \
                and ("/world/%s/" % a.world) in i["cmdline"]:
            hits.append((pid, i, "bridge_exact_world"))
            continue
    actions = []
    for pid, i, why in hits:
        act = {"pid": pid, "why": why, "exe": i["exe_realpath"],
               "cmdline": i["cmdline"][:300], "sid": i["sid"],
               "starttime_s_boot": i["starttime_s_boot"]}
        if a.dry_run:
            act["action"] = "would_kill"
        else:
            r = _signal_verified(pid, signal.SIGTERM, i["starttime_s_boot"])
            act["term"] = r
            if not _wait_gone([pid], 5.0):
                act["kill"] = _signal_verified(pid, signal.SIGKILL, i["starttime_s_boot"])
            act["alive_after"] = alive(pid)
        actions.append(act)
    res = {"scanned_exe": want_exe, "world": a.world, "dry_run": bool(a.dry_run),
           "hits": actions, "count": len(actions)}
    if a.out:
        write_json(a.out, res)
    print(json.dumps(res, ensure_ascii=False, indent=2))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("sid"); p.add_argument("--pid", type=int, default=os.getpid())
    p.set_defaults(fn=cmd_sid)

    p = sub.add_parser("add")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--role", required=True)
    p.add_argument("--pid", type=int, required=True)
    p.add_argument("--note", default="")
    p.set_defaults(fn=cmd_add)

    p = sub.add_parser("scan")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--sid", type=int, required=True)
    p.set_defaults(fn=cmd_scan)

    p = sub.add_parser("watch")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--interval", type=float, default=1.0)
    p.set_defaults(fn=cmd_watch)

    p = sub.add_parser("cleanup")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--sid", type=int, required=True)
    p.add_argument("--reason", default="normal",
                   choices=["normal", "abnormal", "injected", "timeout"])
    p.add_argument("--grace", type=float, default=6.0)
    p.set_defaults(fn=cmd_cleanup)

    p = sub.add_parser("verify")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--sid", type=int, required=True)
    p.set_defaults(fn=cmd_verify)

    p = sub.add_parser("signal-pid")
    p.add_argument("--run-dir", default="")
    p.add_argument("--pid", type=int, required=True)
    p.add_argument("--expect-exe", default="")
    p.add_argument("--expect-starttime", type=int, default=None)
    p.add_argument("--signal", default="TERM", choices=["TERM", "INT", "KILL", "STOP"])
    p.add_argument("--out", default="")
    p.set_defaults(fn=cmd_signal_pid)

    p = sub.add_parser("reclaim")
    p.add_argument("--exe", default="")
    p.add_argument("--world", default="")
    p.add_argument("--out", default="")
    p.add_argument("--dry-run", action="store_true")
    p.set_defaults(fn=cmd_reclaim)

    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
