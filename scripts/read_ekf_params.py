#!/usr/bin/env python3
"""读取 PX4 的 EKF2 视觉融合相关参数（仿真诊断用，不影响飞行）。"""
import sys
import time

from pymavlink import mavutil

KEYS = [
    "EKF2_EV_CTRL", "EKF2_HGT_REF", "EKF2_EV_DELAY", "EKF2_EV_QMIN",
    "EKF2_GPS_CTRL", "EKF2_EV_NOISE_MD", "EKF2_EVP_NOISE", "EKF2_EVV_NOISE",
    "EKF2_EVA_NOISE", "SYS_HAS_GPS", "EKF2_EV_POS_X", "EKF2_EV_POS_Y",
    "EKF2_EV_POS_Z",
]


def main(port="udp:127.0.0.1:14550", wait=30.0):
    m = mavutil.mavlink_connection(port, source_system=250, source_component=190)
    deadline = time.time() + wait
    while time.time() < deadline:
        if m.recv_match(type="HEARTBEAT", blocking=False):
            break
        time.sleep(0.2)
    else:
        print("no heartbeat", flush=True)
        return 1

    wanted = set(KEYS)
    got = {}
    for k in KEYS:
        m.mav.param_request_read_send(1, 1, k.encode(), -1)
    deadline = time.time() + 8.0
    while time.time() < deadline and len(got) < len(wanted):
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode(errors="replace")
        pid = pid.rstrip("\x00")
        if pid in wanted:
            got[pid] = msg.param_value
    for k in KEYS:
        print("%-16s = %s" % (k, got.get(k, "?")))
    return 0


if __name__ == "__main__":
    sys.exit(main(*(sys.argv[1:] or [])))
