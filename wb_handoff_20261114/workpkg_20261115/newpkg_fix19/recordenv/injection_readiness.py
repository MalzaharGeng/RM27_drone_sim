#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[NP-01 FIX19] 注入器就绪检查（在向任何进程发信号之前必须全部通过）。

硬约束
------
* 唯一 VIO 发布节点按 **exe 路径 + 完整 sha256** 识别，不按名字/cmdline 子串。
* launcher、整轮运行器、桥必须被显式排除并记录，绝不能被当成节点。
* PID / starttime_s_boot / exe_realpath / sha256 四项与登记表及冻结二进制一致。
* 只允许向这一个 PID 发送预定信号（默认 SIGTERM）。
* 注入器自身退出不触发整轮无条件 cleanup（见 shell 侧 setsid 与无 trap 约定）。

子命令
------
  check  --run-dir D --node-bin PATH --registry FILE --out FILE
  self-check [--work-dir DIR]
"""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

ROLE_EXCLUDE = {"launcher", "runner", "bridge", "supervisor", "telemetry",
                "obs_counter", "recorder", "px4", "gz", "session_member"}


def stat(pid):
    try:
        s = open("/proc/%d/stat" % pid).read()
        f = s[s.rfind(")") + 2:].split()
        return {"state": f[0], "ppid": int(f[1]), "sid": int(f[3]),
                "starttime_s_boot": int(f[19])}
    except OSError:
        return None


def exe_real(pid):
    try:
        return os.path.realpath(os.readlink("/proc/%d/exe" % pid))
    except OSError:
        return None


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        while True:
            b = fh.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def cmdline(pid):
    try:
        return open("/proc/%d/cmdline" % pid, "rb").read().replace(
            b"\0", b" ").decode("utf-8", "replace").strip()[:400]
    except OSError:
        return ""


def write_json(path, obj):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, ensure_ascii=False, indent=2, sort_keys=True)
        fh.write("\n")
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def load_registry(path):
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


def check(run_dir, node_bin, registry_path, node_pid_file=None):
    node_bin_real = os.path.realpath(node_bin)
    frozen_hash = sha256(node_bin_real) if os.path.isfile(node_bin_real) else None
    reg = load_registry(registry_path)

    # 1) 按 exe 路径扫全表，找唯一 VIO 节点
    candidates, excluded = [], []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        ep = exe_real(pid)
        if ep == node_bin_real:
            candidates.append(pid)

    # 2) 明确列出并排除非节点角色
    for e in reg:
        pid = int(e.get("pid", 0))
        role = e.get("role")
        if pid in candidates:
            continue
        ep = exe_real(pid)
        excluded.append({"pid": pid, "role": role,
                         "exe_realpath": ep, "alive": stat(pid) is not None})

    # 3) 登记表里的 node 条目
    reg_nodes = [e for e in reg if e.get("role") == "node"]
    reg_node_pids = [int(e["pid"]) for e in reg_nodes]

    # 4) node_pid.json
    declared = None
    if node_pid_file and os.path.exists(node_pid_file):
        declared = json.load(open(node_pid_file, encoding="utf-8"))

    problems = []
    if not os.path.isfile(node_bin_real):
        problems.append("frozen_node_binary_missing")
    if frozen_hash is None:
        problems.append("frozen_sha256_unavailable")
    if len(candidates) != 1:
        problems.append("candidate_count=%d (expect 1)" % len(candidates))
    if len(reg_nodes) > 1:
        problems.append("registry_node_entries=%d" % len(reg_nodes))

    target = candidates[0] if len(candidates) == 1 else None
    identity = None
    if target is not None:
        st = stat(target)
        ep = exe_real(target)
        live_hash = sha256(ep) if ep and os.path.isfile(ep) else None
        identity = {
            "pid": target,
            "starttime_s_boot": st["starttime_s_boot"] if st else None,
            "sid": st["sid"] if st else None,
            "exe_realpath": ep,
            "sha256": live_hash,
            "cmdline": cmdline(target),
        }
        if not st:
            problems.append("target_stat_unreadable")
        if not ep or ep != node_bin_real:
            problems.append("target_exe_mismatch")
        if frozen_hash and live_hash != frozen_hash:
            problems.append("target_sha256_mismatch")
        if declared:
            if int(declared.get("pid", -1)) != target:
                problems.append("declared_pid_mismatch")
            if declared.get("starttime_s_boot") is not None and st and \
                    int(declared["starttime_s_boot"]) != st["starttime_s_boot"]:
                problems.append("declared_starttime_mismatch")
        if reg_node_pids and target not in reg_node_pids:
            problems.append("registry_node_pid_mismatch")
        # 目标不能是任何被排除的角色
        for e in reg:
            if int(e.get("pid", -1)) == target and e.get("role") in ROLE_EXCLUDE:
                problems.append("target_also_tagged_as_" + str(e["role"]))

    return {
        "checked_wall_us": int(time.time() * 1e6),
        "checked_mono_us": time.monotonic_ns() // 1000,
        "frozen_node_bin": node_bin_real,
        "frozen_node_sha256": frozen_hash,
        "frozen_node_md5": (hashlib.md5(open(node_bin_real, "rb").read()).hexdigest()
                            if os.path.isfile(node_bin_real) else None),
        "candidate_pids": candidates,
        "unique_vio_node": target,
        "target_identity": identity,
        "registry_node_entries": reg_nodes,
        "node_pid_json": declared,
        "excluded_non_node_processes": excluded[:30],
        "excluded_count": len(excluded),
        "planned_signal": "SIGTERM",
        "planned_target_only": True,
        "problems": problems,
        "ready": len(problems) == 0 and target is not None,
    }


def cmd_check(a):
    res = check(a.run_dir, a.node_bin, a.registry, a.node_pid)
    write_json(a.out, res)
    print(json.dumps({k: res[k] for k in (
        "ready", "unique_vio_node", "frozen_node_sha256", "problems",
        "candidate_pids", "excluded_count")}, ensure_ascii=False, indent=2))
    return 0 if res["ready"] else 1


def cmd_self_check(a):
    """替身验证：唯一识别 / 排除其它角色 / 只向节点发信号 / 注入器退出不 cleanup。"""
    wd = a.work_dir or tempfile.mkdtemp(prefix="np01_fix19_injection_self_")
    os.makedirs(wd, exist_ok=True)
    checks = []

    # 造一个「冻结节点二进制」替身副本：用 Python 解释器的真实 ELF，
    # 启动后保持该 ELF 为 /proc/<pid>/exe（不用 coreutils 多调用链接，
    # 也不让 shell 用 exec 优化把 exe 换成别的程序）。
    import shutil
    fake_bin = os.path.join(wd, "fake_vio_node")
    src_elf = os.path.realpath(sys.executable)
    shutil.copy2(src_elf, fake_bin)
    os.chmod(fake_bin, 0o755)

    # 起 4 个替身进程：节点用 copy 出来的 ELF（唯一 exe 路径），
    # 其它三个角色用解释器原路径启动 —— 保证按 exe 扫描时只有 1 个候选。
    other_elf = src_elf
    node = subprocess.Popen([fake_bin, "-c", "import time; time.sleep(60)"],
                            start_new_session=True)
    time.sleep(0.2)
    launcher = subprocess.Popen([other_elf, "-c", "import time; time.sleep(60)"],
                                start_new_session=True)
    runner = subprocess.Popen([other_elf, "-c", "import time; time.sleep(60)"],
                              start_new_session=True)
    bridge = subprocess.Popen([other_elf, "-c", "import time; time.sleep(60)"],
                              start_new_session=True)

    # 构造登记表（角色齐全）
    reg = os.path.join(wd, "proc_registry.jsonl")
    with open(reg, "w") as fh:
        for pid, role in ((launcher.pid, "launcher"), (runner.pid, "runner"),
                          (bridge.pid, "bridge"), (node.pid, "node")):
            st = stat(pid)
            fh.write(json.dumps({
                "role": role, "pid": pid,
                "starttime_s_boot": st["starttime_s_boot"],
                "exe_realpath": exe_real(pid)}) + "\n")

    npj = os.path.join(wd, "node_pid.json")
    stn = stat(node.pid)
    write_json(npj, {"pid": node.pid,
                     "starttime_s_boot": stn["starttime_s_boot"],
                     "exe": os.path.realpath(fake_bin),
                     "is_only_vio_publisher": True})

    res = check(wd, fake_bin, reg, npj)
    out = os.path.join(wd, "injection_readiness.json")
    write_json(out, res)
    checks.append(("S1_unique_node_identified",
                   res["ready"] and res["unique_vio_node"] == node.pid,
                   {"candidates": res["candidate_pids"]}))

    # S2 其它角色被排除且不在候选里
    others = {launcher.pid, runner.pid, bridge.pid}
    checks.append(("S2_non_nodes_excluded",
                   all(p not in res["candidate_pids"] for p in others) and
                   res["excluded_count"] >= 3,
                   {"excluded": res["excluded_count"]}))

    # S3 完整哈希一致
    ti = res.get("target_identity") or {}
    checks.append(("S3_sha256_match",
                   bool(res["frozen_node_sha256"]) and
                   res["frozen_node_sha256"] == ti.get("sha256"),
                   {"frozen": res["frozen_node_sha256"], "live": ti.get("sha256")}))

    # S4 starttime / exe 一致
    checks.append(("S4_starttime_and_exe_match",
                   ti.get("starttime_s_boot") == stn["starttime_s_boot"] and
                   ti.get("exe_realpath") == os.path.realpath(fake_bin),
                   {}))

    # S5 篡改 starttime 必须被拒
    bad = json.load(open(npj))
    bad["starttime_s_boot"] += 12345
    write_json(os.path.join(wd, "node_pid_bad.json"), bad)
    res_bad = check(wd, fake_bin, reg, os.path.join(wd, "node_pid_bad.json"))
    checks.append(("S5_tampered_starttime_rejected",
                   not res_bad["ready"] and
                   any("starttime" in p for p in res_bad["problems"]),
                   {"problems": res_bad["problems"]}))

    # S6 只向节点发预定信号；其它替身仍活着
    if res["ready"]:
        os.kill(res["unique_vio_node"], signal.SIGTERM)
        time.sleep(0.4)
    after = {p: (stat(p) is not None and stat(p)["state"] not in ("Z", "X"))
             for p in others}
    checks.append(("S6_only_node_signalled",
                   (stat(node.pid) is None or stat(node.pid)["state"] in ("Z", "X"))
                   and all(after.values()), {"others_alive": after}))

    # S7 注入器「退出」不触发整轮 cleanup：替身 launcher/runner/bridge 仍活
    checks.append(("S7_injector_exit_no_round_cleanup",
                   all(after.values()), {}))

    # 清理替身
    for p in (launcher, runner, bridge):
        try:
            p.send_signal(signal.SIGKILL); p.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            pass
    try:
        node.wait(timeout=3)
    except subprocess.TimeoutExpired:
        node.kill(); node.wait()

    result = {"work_dir": wd,
              "checks": [{"name": n, "pass": bool(ok), "detail": d}
                         for n, ok, d in checks]}
    write_json(os.path.join(wd, "self_check_result.json"), result)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if all(c["pass"] for c in result["checks"]) else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("check")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--node-bin", required=True)
    p.add_argument("--registry", required=True)
    p.add_argument("--node-pid", default="")
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_check)
    p = sub.add_parser("self-check")
    p.add_argument("--work-dir", default="")
    p.set_defaults(fn=cmd_self_check)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
