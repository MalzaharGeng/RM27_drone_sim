#!/usr/bin/env python3
"""PX4/Gazebo stdout 限流：滤掉无换行的 pxh> 提示符噪声，实时写盘并设容量上限。

PX4 SITL 在非交互 shell 下会持续把 "pxh> " 写到 stdout 且不换行，
单轮可产生 1 GB 以上日志。此工具只做输出整理，不参与控制或判定。
输出实时可见（runner 依赖日志中的 gz_bridge 就绪行），超限后继续消费
stdin 但不写盘，避免给 PX4 造成 SIGPIPE。
"""
import argparse
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-mb", type=float, default=96.0)
    args = ap.parse_args()
    cap = int(args.max_mb * 1024 * 1024)
    token = b"pxh> "
    carry = b""
    total = kept = 0
    with open(args.out, "wb", buffering=0) as f:
        # read1() returns whatever one underlying read yields instead of blocking
        # until 64 KiB accumulates. PX4 is quiet when EKF is healthy, so read()
        # could leave px4_gz.log empty and make the runner miss the gz_bridge
        # readiness line. Log-tool fix only; no gating/parameter change.
        for chunk in iter(lambda: sys.stdin.buffer.read1(65536), b""):
            data = (carry + chunk).replace(token, b"")
            carry = b""
            for k in range(len(token) - 1, 0, -1):
                if data.endswith(token[:k]):
                    carry, data = data[-k:], data[:-k]
                    break
            if not data:
                continue
            total += len(data)
            if kept < cap:
                take = min(cap - kept, len(data))
                f.write(data[:take])
                kept += take
        if carry:
            total += len(carry)
            if kept < cap:
                take = min(cap - kept, len(carry))
                f.write(carry[:take])
                kept += take
        if total > kept:
            f.write(("\n[log_cap] output truncated: raw=%d kept=%d dropped=%d bytes\n"
                     % (total, kept, total - kept)).encode())
    sys.stderr.write("log_cap: raw=%d kept=%d dropped=%d out=%s\n"
                     % (total, kept, total - kept, args.out))


if __name__ == "__main__":
    main()
