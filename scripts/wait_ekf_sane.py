#!/usr/bin/env python3
"""启动飞手前的 EKF 合理性检查（有界、fail-closed）。

背景（phase3 run 20261114_215209）：PX4 冷启动后、飞手连接前，EKF2 就已经
发散——telemetry 第一帧 lp=(-0.21,-10.97,9.42)，30 s 内到 km 级；随后
arming 被 "ekf2 missing data" 拒绝。飞手在 13 s 后才启动，不可能造成该发散。
如果不加检查，这类启动故障会被误读成"飞手修复失败"。

本脚本只读本轮 telemetry.csv，等采样窗口足够长后检查最后若干个样本：
  * x/y/z 有限且不超过 --max-pos（默认 20 m，起飞前 EKF 应停在 0 附近）；
  * 速度有限且不超过 --max-vel（默认 2 m/s，起飞前飞机静止）。
超时或超限即退出 1。不改变任何门控/验收标准，只决定"本轮是否值得启动飞手"。
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path


def fnum(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--telemetry", required=True)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--min-span", type=float, default=4.0)
    ap.add_argument("--rows", type=int, default=10)
    # 只看启动窗口：默认 0=全部行（运行中调用时本来就只有启动数据）。
    # 对历史 telemetry 复检时必须限制窗口，否则会把飞行后的失跟踪当成启动故障。
    ap.add_argument("--within", type=float, default=0.0)
    ap.add_argument("--max-pos", type=float, default=20.0)
    ap.add_argument("--max-vel", type=float, default=2.0)
    args = ap.parse_args()

    path = Path(args.telemetry)
    deadline = time.monotonic() + args.timeout
    last = None
    while True:
        rows = []
        if path.exists():
            try:
                with path.open(newline="") as f:
                    rows = list(csv.DictReader(f))
            except (OSError, csv.Error):
                rows = []
        if args.within > 0:
            rows = [r for r in rows
                    if (fnum(r.get("elapsed_s")) is not None
                        and fnum(r.get("elapsed_s")) <= args.within)]
        if len(rows) >= args.rows:
            first = fnum(rows[0].get("elapsed_s"))
            last = fnum(rows[-1].get("elapsed_s"))
            if first is not None and last is not None and last - first >= args.min_span:
                break
        if time.monotonic() >= deadline:
            print("EKF_SANITY=FAIL timeout rows=%d" % len(rows))
            return 1
        time.sleep(0.2)

    tail = rows[-args.rows:]
    worst = {"pos": 0.0, "vel": 0.0}
    for r in tail:
        vals = [fnum(r.get(k)) for k in
                ("lp_x", "lp_y", "lp_z", "lp_vx", "lp_vy", "lp_vz")]
        if any(v is None for v in vals):
            print("EKF_SANITY=FAIL non-finite sample: %r" % (r,))
            return 1
        pos = max(abs(vals[0]), abs(vals[1]), abs(vals[2]))
        vel = max(abs(vals[3]), abs(vals[4]), abs(vals[5]))
        worst["pos"] = max(worst["pos"], pos)
        worst["vel"] = max(worst["vel"], vel)

    ok = worst["pos"] <= args.max_pos and worst["vel"] <= args.max_vel
    print("EKF_SANITY=%s span=%.1fs rows=%d max_pos=%.2fm(max %.1f) "
          "max_vel=%.2fm/s(max %.1f) last_lp=(%.2f,%.2f,%.2f)"
          % ("PASS" if ok else "FAIL", last - fnum(rows[0]["elapsed_s"]), len(rows),
             worst["pos"], args.max_pos, worst["vel"], args.max_vel,
             fnum(tail[-1]["lp_x"]), fnum(tail[-1]["lp_y"]),
             fnum(tail[-1]["lp_z"])))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
