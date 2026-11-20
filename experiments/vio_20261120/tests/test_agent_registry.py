#!/usr/bin/env python3
"""agent_registry 的故障注入测试：pid 复用、自建会话、启动失败、非本轮启动。

不依赖 ROS/PX4；用 `sleep`/`setsid sleep` 充当替身进程。
"""
import json
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import time
import unittest

TOOL = str(Path(__file__).resolve().parents[1] / "runner/agent_registry.py")


def run(*args):
    p = subprocess.run([sys.executable, TOOL] + list(args),
                       capture_output=True, text=True)
    out = p.stdout.strip().splitlines()
    return p.returncode, (json.loads(out[-1]) if out else None)


def alive(pid):
    """存活 = /proc 存在且不是僵尸（僵尸已死，只是未被 reap）。"""
    try:
        with open("/proc/%d/stat" % pid) as f:
            raw = f.read()
    except OSError:
        return False
    idx = raw.rfind(")")
    if idx < 0:
        return False
    fields = raw[idx + 2:].split()
    return bool(fields) and fields[0] != "Z"


class AgentRegistryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="agentreg_")
        self.reg = os.path.join(self.tmp, "agent_owner.json")
        self.procs = []

    def tearDown(self):
        for p in self.procs:
            try:
                p.kill()
                p.wait(timeout=2)
            except Exception:
                pass

    def _spawn(self, setsid=False):
        """替身进程：setsid=True 时由子进程自己调用 os.setsid()，
        这样 pid 就是会话首进程（sid == pid），且 pid 稳定可查。
        不用 /usr/bin/setsid —— 它在已是会话首进程时会 fork 并退出，
        父 pid 会被回收复用，导致测试自身失真。"""
        if setsid:
            code = "import os,time;os.setsid();time.sleep(60)"
            p = subprocess.Popen([sys.executable, "-c", code])
        else:
            p = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(60)"])
        self.procs.append(p)
        time.sleep(0.25)
        return p

    # 1) 启动失败：pid=0 时登记必须可用但不回收
    def test_start_failure_records_unavailable_and_skips(self):
        rc, rec = run("record", "--out", self.reg, "--binary", sys.executable,
                      "--pid", "0", "--started-by-us", "1", "--round-sid", "1")
        self.assertEqual(rc, 0)
        self.assertFalse(rec["available"])
        rc, res = run("reclaim", "--registry", self.reg)
        self.assertEqual(rc, 0)
        self.assertEqual(res["action"], "skip_unavailable")

    # 2) 自建会话：setsid 后 sid == 自身 pid（回收不能依赖 sid 父子关系）
    def test_self_session_is_recorded(self):
        p = self._spawn(setsid=True)
        rc, rec = run("record", "--out", self.reg, "--binary", sys.executable,
                      "--pid", str(p.pid), "--started-by-us", "1", "--round-sid", "1")
        self.assertEqual(rc, 0)
        self.assertTrue(rec["available"])
        self.assertEqual(rec["sid"], p.pid, "setsid 后 sid 应等于自身 pid")
        self.assertNotEqual(rec["sid"], os.getsid(0), "不应与本测试进程同会话")

    # 3) 正常回收：身份匹配则 SIGTERM 成功
    def test_reclaim_terminates_matching_identity(self):
        p = self._spawn(setsid=True)
        run("record", "--out", self.reg, "--binary", sys.executable,
            "--pid", str(p.pid), "--started-by-us", "1", "--round-sid", "1")
        rc, res = run("reclaim", "--registry", self.reg, "--grace", "2.0")
        self.assertEqual(rc, 0)
        self.assertEqual(res["action"], "terminated")
        time.sleep(0.2)
        self.assertFalse(alive(p.pid))

    # 4) pid 复用：登记后 starttime 变化 -> 必须跳过，绝不误杀
    def test_pid_reuse_is_skipped(self):
        p = self._spawn(setsid=True)
        run("record", "--out", self.reg, "--binary", sys.executable,
            "--pid", str(p.pid), "--started-by-us", "1", "--round-sid", "1")
        rec = json.load(open(self.reg))
        rec["starttime"] = int(rec["starttime"]) - 12345   # 伪造：不是同一进程
        json.dump(rec, open(self.reg, "w"))
        rc, res = run("reclaim", "--registry", self.reg)
        self.assertEqual(rc, 0)
        self.assertEqual(res["action"], "skip_pid_reused")
        self.assertTrue(alive(p.pid), "pid 复用时必须保留进程")

    # 5) 非本轮启动：started_by_us=0 -> 不回收（保护其他任务）
    def test_not_started_by_us_is_skipped(self):
        p = self._spawn(setsid=True)
        run("record", "--out", self.reg, "--binary", sys.executable,
            "--pid", str(p.pid), "--started-by-us", "0", "--round-sid", "1")
        rc, res = run("reclaim", "--registry", self.reg)
        self.assertEqual(rc, 0)
        self.assertEqual(res["action"], "skip_not_started_by_us")
        self.assertTrue(alive(p.pid))

    # 6) 已消失：进程先退出，回收应报 already_gone，不报错
    def test_already_gone(self):
        p = self._spawn(setsid=True)
        run("record", "--out", self.reg, "--binary", sys.executable,
            "--pid", str(p.pid), "--started-by-us", "1", "--round-sid", "1")
        p.kill()
        p.wait(timeout=2)
        rc, res = run("reclaim", "--registry", self.reg)
        self.assertEqual(rc, 0)
        self.assertIn(res["action"], ("already_gone", "skip_pid_reused"))

    # 7) 无登记文件：跳过而不是崩
    def test_missing_registry(self):
        rc, res = run("reclaim", "--registry", os.path.join(self.tmp, "nope.json"))
        self.assertEqual(rc, 0)
        self.assertEqual(res["action"], "skip_no_registry")

    # 8) 命令不匹配：同名 pid 但 cmdline 不含登记二进制 -> 跳过
    def test_cmdline_mismatch_is_skipped(self):
        p = self._spawn(setsid=True)
        run("record", "--out", self.reg, "--binary", "MicroXRCEAgent",
            "--pid", str(p.pid), "--started-by-us", "1", "--round-sid", "1")
        rc, res = run("reclaim", "--registry", self.reg)
        self.assertEqual(rc, 0)
        self.assertEqual(res["action"], "skip_cmdline_mismatch")
        self.assertTrue(alive(p.pid))

    # 9) scan 能识别 agent 关键字与自身字段
    def test_scan_shape(self):
        rc, out = run("scan")
        self.assertEqual(rc, 0)
        self.assertIn("found", out)
        self.assertIsInstance(out["found"], list)


if __name__ == "__main__":
    unittest.main(verbosity=2)
