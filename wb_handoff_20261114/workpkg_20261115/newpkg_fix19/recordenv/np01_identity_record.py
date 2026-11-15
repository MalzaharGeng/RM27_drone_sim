#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX19 记录修订版] 进程身份证据记录（纯证据层，不参与任何控制）。

为什么存在
----------
FIX18 的两段内联证据代码里用了 open("/proc/<pid>/exe", "rb").read()：
  * supervisor_pid.json 把可执行文件二进制（python3.14，25.88 MB）当字符串写进
    JSON 字段；
  * bridge_owner.json 又把这段二进制当作「路径」去 open 求 md5，异常被
    `2>/dev/null || true` 吞掉，文件根本没生成。

本模块把身份记录收敛到一个地方，硬约束：
  1. exe 只取**路径字符串**：os.readlink("/proc/<pid>/exe") + os.path.realpath()；
     绝不读取 /proc/<pid>/exe 指向的二进制内容作为字段。
  2. 需要「完整哈希」时，对 os.path.realpath 得到的**磁盘文件**计算
     sha256/md5，只把 hex digest（64/32 字节）写入 JSON，不写字节内容。
  3. 所有字段有类型与长度上限；写盘用临时文件 + fsync + os.replace 原子替换。
  4. 进程已退出 / /proc 条目不可读时，返回结构化缺失记录，不抛异常、不写空壳。
  5. 输出文件名由调用方指定（*.fix19.json），不覆盖任何历史证据。

子命令
------
  bridge-owner   --pid --run-dir --log --left --right --imu --truth --out
  supervisor     --pid --run-dir --out
  node           --pid --run-dir --out [--expect-exe PATH]
  verify         --record FILE [--pid PID]
  self-test      [--work-dir DIR]     替身进程验证（身份/序列化/缺失/清理）
"""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

MAX_STR = 1500
MAX_JSON_BYTES = 65536


def _safe_str(v, limit=MAX_STR):
    if v is None:
        return None
    if not isinstance(v, str):
        v = str(v)
    # 只保留可打印字符，避免把二进制内容混进 JSON
    v = "".join(c if (32 <= ord(c) < 127 or c in "\t\n") else "?" for c in v)
    return v[:limit]


def read_stat(pid):
    """返回 (state, ppid, sid, starttime_s_boot)；缺失返回 None。"""
    try:
        s = open("/proc/%d/stat" % pid, "r").read()
    except OSError:
        return None
    try:
        f = s[s.rfind(")") + 2:].split()
        return {"state": f[0], "ppid": int(f[1]), "sid": int(f[3]),
                "starttime_s_boot": int(f[19])}
    except (IndexError, ValueError):
        return None


def exe_path(pid):
    """只取路径字符串，绝不读取 /proc/<pid>/exe 的内容。"""
    link = "/proc/%d/exe" % pid
    try:
        target = os.readlink(link)
    except OSError as exc:
        return {"available": False, "reason": "readlink_failed:%s" % exc.__class__.__name__,
                "link": link}
    real = os.path.realpath(target)
    return {"available": True, "link": link, "link_target": _safe_str(target, 512),
            "exe_realpath": _safe_str(real, 512),
            "exists": os.path.isfile(real),
            "note": "path only; binary contents are NEVER read as a field"}


def file_hashes(path):
    """对真实磁盘文件算完整哈希；只返回 hex digest。"""
    if not path or not os.path.isfile(path):
        return {"available": False, "reason": "exe_file_missing"}
    try:
        size = os.path.getsize(path)
        h5, h256 = hashlib.md5(), hashlib.sha256()
        with open(path, "rb") as fh:
            while True:
                b = fh.read(1 << 20)
                if not b:
                    break
                h5.update(b)
                h256.update(b)
        return {"available": True, "size_bytes": size,
                "md5": h5.hexdigest(), "sha256": h256.hexdigest()}
    except OSError as exc:
        return {"available": False, "reason": "hash_failed:%s" % exc.__class__.__name__}


def cmdline(pid):
    try:
        raw = open("/proc/%d/cmdline" % pid, "rb").read()
    except OSError:
        return None
    return _safe_str(raw.replace(b"\0", b" ").decode("utf-8", "replace").strip(), MAX_STR)


def loaded_libs(pid, needle=".so"):
    out = []
    try:
        with open("/proc/%d/maps" % pid, "r", errors="replace") as fh:
            for line in fh:
                parts = line.split()
                if len(parts) > 5 and parts[-1].endswith(needle):
                    out.append(parts[-1])
    except OSError:
        pass
    return sorted(set(out))[:12]


def identity(pid):
    st = read_stat(pid)
    if st is None:
        return {"pid": int(pid), "available": False,
                "reason": "process_missing_or_stat_unreadable",
                "recorded_wall_us": int(time.time() * 1e6)}
    ep = exe_path(pid)
    rec = {
        "pid": int(pid), "available": True,
        "recorded_wall_us": int(time.time() * 1e6),
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "state": st["state"], "ppid": st["ppid"], "sid": st["sid"],
        "starttime_s_boot": st["starttime_s_boot"],
        "cmdline": cmdline(pid),
        "exe": ep,
    }
    if ep.get("available") and ep.get("exists"):
        rec["exe_file_hash"] = file_hashes(ep["exe_realpath"])
    else:
        rec["exe_file_hash"] = {"available": False,
                               "reason": "no_realpath_to_hash"}
    return rec


def write_json(path, obj):
    """原子写 + fsync；序列化后做大小与类型护栏，杜绝 25MB 事故重演。"""
    text = json.dumps(obj, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    raw = text.encode("utf-8")
    if len(raw) > MAX_JSON_BYTES:
        raise ValueError("refuse to write oversized evidence (%d bytes > %d)"
                         % (len(raw), MAX_JSON_BYTES))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(text)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)
    return len(raw)


def build(kind, pid, extra=None):
    rec = identity(pid)
    rec["record_kind"] = kind
    rec["record_revision"] = "fix19"
    if extra:
        rec.update(extra)
    return rec


def cmd_bridge_owner(a):
    rec = build("bridge_owner", a.pid, {
        "owner": "run_profile_fix19.sh (whole-round runner)",
        "log_path": _safe_str(a.log, 512),
        "loaded_ros_gz_bridge_so": [p for p in loaded_libs(a.pid)
                                    if "ros_gz_bridge" in p][:8],
        "topics": {"left": a.left, "right": a.right, "imu": a.imu,
                   "truth": a.truth, "clock": "/clock"},
        "child_parameter_bridge_pids": _child_bridge_pids(a.pid),
    })
    n = write_json(a.out, rec)
    print(json.dumps({"out": a.out, "bytes": n, "pid": a.pid,
                      "available": rec["available"],
                      "exe_realpath": rec.get("exe", {}).get("exe_realpath")},
                     ensure_ascii=False))
    return 0


def _child_bridge_pids(ppid):
    """ros2 CLI 会派生真正的 parameter_bridge 子进程；按 ppid 精确列出。"""
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        st = read_stat(pid)
        if not st or st["ppid"] != ppid:
            continue
        ep = exe_path(pid)
        if ep.get("available") and "parameter_bridge" in ep.get("exe_realpath", ""):
            out.append({"pid": pid, "starttime_s_boot": st["starttime_s_boot"],
                        "exe_realpath": ep["exe_realpath"]})
    return out


def cmd_supervisor(a):
    rec = build("supervisor", a.pid)
    n = write_json(a.out, rec)
    print(json.dumps({"out": a.out, "bytes": n, "pid": a.pid,
                      "exe_realpath": rec.get("exe", {}).get("exe_realpath")},
                     ensure_ascii=False))
    return 0


def cmd_node(a):
    rec = build("node", a.pid, {"is_only_vio_publisher": True})
    if a.expect_exe:
        want = os.path.realpath(a.expect_exe)
        got = json.loads(json.dumps(rec.get("exe") or {}))
        rec["expect_exe_realpath"] = want
        rec["exe_match"] = (got.get("exe_realpath") == want)
        rec["expect_exe_hash"] = file_hashes(want)
        rec["hash_match"] = (rec["expect_exe_hash"].get("sha256")
                             == (rec.get("exe_file_hash") or {}).get("sha256"))
    n = write_json(a.out, rec)
    print(json.dumps({"out": a.out, "bytes": n, "pid": a.pid,
                      "exe_match": rec.get("exe_match"),
                      "hash_match": rec.get("hash_match")}, ensure_ascii=False))
    return 0


def cmd_verify(a):
    rec = json.load(open(a.record, encoding="utf-8"))
    pid = a.pid if a.pid is not None else int(rec.get("pid", 0))
    cur = identity(pid)
    ok = True
    notes = []
    if not cur.get("available"):
        ok = False
        notes.append("process_not_alive")
    else:
        if rec.get("starttime_s_boot") != cur.get("starttime_s_boot"):
            ok = False
            notes.append("starttime_mismatch")
        old = (rec.get("exe") or {}).get("exe_realpath")
        new = (cur.get("exe") or {}).get("exe_realpath")
        if old and old != new:
            ok = False
            notes.append("exe_realpath_mismatch")
    print(json.dumps({"record": a.record, "pid": pid, "identity_ok": ok,
                      "notes": notes,
                      "record_bytes": os.path.getsize(a.record)},
                     ensure_ascii=False))
    return 0 if ok else 1


def cmd_self_test(a):
    """替身验证：身份记录 / 序列化护栏 / 缺失路径 / 正常清理。"""
    import tempfile
    wd = a.work_dir or tempfile.mkdtemp(prefix="np01_fix19_record_self_test_")
    os.makedirs(wd, exist_ok=True)
    results = []

    # 起一个替身进程（独立会话），模拟 bridge/supervisor
    p = subprocess.Popen(["/bin/sleep", "30"], start_new_session=True)
    time.sleep(0.2)
    pid = p.pid

    # T1 身份记录 + 序列化
    rec = build("self_test", pid)
    f1 = os.path.join(wd, "identity.json")
    n = write_json(f1, rec)
    results.append(("T1_identity_and_size", n < MAX_JSON_BYTES and n < 8192,
                    {"bytes": n, "exe_realpath": (rec.get("exe") or {}).get("exe_realpath")}))
    reread = json.load(open(f1, encoding="utf-8"))
    results.append(("T2_json_roundtrip", reread["pid"] == pid and
                    reread["starttime_s_boot"] == rec["starttime_s_boot"], {}))
    # T3 明确禁止把二进制内容写进字段：exe 字段必须是 dict 且不含长二进制串
    exe_field = reread.get("exe") or {}
    results.append(("T3_no_binary_in_field",
                    isinstance(exe_field, dict) and not isinstance(exe_field, str) and
                    len(str(exe_field)) < 2048, {"exe_field_len": len(str(exe_field))}))
    # T4 verify 能核对身份
    vr = subprocess.run([sys.executable, os.path.abspath(__file__), "verify",
                         "--record", f1, "--pid", str(pid)],
                        capture_output=True, text=True)
    results.append(("T4_verify_ok", vr.returncode == 0, {"stdout": vr.stdout.strip()[:160]}))
    # T5 身份篡改（错误 starttime）必须被 verify 拒绝
    bad = dict(reread)
    bad["starttime_s_boot"] = int(reread["starttime_s_boot"]) + 12345
    f2 = os.path.join(wd, "identity_bad.json")
    write_json(f2, bad)
    vr2 = subprocess.run([sys.executable, os.path.abspath(__file__), "verify",
                          "--record", f2, "--pid", str(pid)],
                         capture_output=True, text=True)
    results.append(("T5_tamper_rejected", vr2.returncode != 0, {}))

    # T6 缺失路径：进程退出后不抛异常、不写二进制
    p.send_signal(signal.SIGTERM)
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill(); p.wait()
    time.sleep(0.3)
    gone = identity(pid)
    f3 = os.path.join(wd, "identity_missing.json")
    n3 = write_json(f3, {"record_kind": "missing_path", "identity": gone})
    results.append(("T6_missing_path_structured",
                    gone.get("available") is False and n3 < 8192 and
                    "reason" in gone, {"reason": gone.get("reason"), "bytes": n3}))

    # T7 正常清理：用 FIX18 的 np01_lifecycle cleanup 复核替身会话（归属不变）
    lc = ("/home/gengyinghao/drone_sim/wb_handoff_20261114/workpkg_20261115/"
          "newpkg_fix18/runenv/np01_lifecycle.py")
    sid = rec.get("sid")
    if os.path.exists(lc) and sid:
        cp = subprocess.run([sys.executable, lc, "cleanup", "--run-dir", wd,
                             "--sid", str(sid), "--reason", "normal", "--grace", "3"],
                            capture_output=True, text=True)
        cj = os.path.join(wd, "process_cleanup.json")
        ok = cp.returncode == 0 and os.path.exists(cj) and \
            json.load(open(cj)).get("verified_empty") is True
        results.append(("T7_cleanup_verified_empty", ok,
                        {"stdout": cp.stdout.strip()[:160]}))
    else:
        results.append(("T7_cleanup_verified_empty", False, {"reason": "lifecycle_missing"}))

    out = {"work_dir": wd, "checks": [
        {"name": nm, "pass": bool(ok), "detail": det} for nm, ok, det in results]}
    write_json(os.path.join(wd, "self_test_result.json"), out)
    print(json.dumps(out, ensure_ascii=False, indent=2))
    failed = [c["name"] for c in out["checks"] if not c["pass"]]
    return 0 if not failed else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("bridge-owner")
    p.add_argument("--pid", type=int, required=True)
    p.add_argument("--run-dir", default="")
    p.add_argument("--log", default="")
    p.add_argument("--left", default="")
    p.add_argument("--right", default="")
    p.add_argument("--imu", default="")
    p.add_argument("--truth", default="")
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_bridge_owner)

    p = sub.add_parser("supervisor")
    p.add_argument("--pid", type=int, required=True)
    p.add_argument("--run-dir", default="")
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_supervisor)

    p = sub.add_parser("node")
    p.add_argument("--pid", type=int, required=True)
    p.add_argument("--run-dir", default="")
    p.add_argument("--out", required=True)
    p.add_argument("--expect-exe", default="")
    p.set_defaults(fn=cmd_node)

    p = sub.add_parser("verify")
    p.add_argument("--record", required=True)
    p.add_argument("--pid", type=int, default=None)
    p.set_defaults(fn=cmd_verify)

    p = sub.add_parser("self-test")
    p.add_argument("--work-dir", default="")
    p.set_defaults(fn=cmd_self_test)

    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
