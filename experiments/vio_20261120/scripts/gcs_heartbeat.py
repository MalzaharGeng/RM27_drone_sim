#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最小 GCS：在 UDP 14550 监听 PX4 遥测并回发 GCS 心跳。

PX4 的 Normal MAVLink 链是「本地 18570 → 远端 14550」，所以必须
**监听** 14550，只往 14550 发包是收不到 PX4 数据的。没有 GCS 心跳时
`NAV_DLL_ACT > 0` 会让前检报 `No connection to the GCS`，无法解锁。

用法：
    python3 scripts/gcs_heartbeat.py            # 只维持心跳
    python3 scripts/gcs_heartbeat.py loiter     # 顺手切回 AUTO_LOITER
"""

import sys
import time

from pymavlink import mavutil


def main():
    m = mavutil.mavlink_connection("udpin:0.0.0.0:14550",
                                   source_system=255, source_component=190)
    print("listening on udp 14550", flush=True)
    hb = m.wait_heartbeat(timeout=30)
    print("PX4 heartbeat ok (system %d)" % hb.get_srcSystem(), flush=True)

    def heartbeat():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    if len(sys.argv) > 1 and sys.argv[1] == "loiter":
        # MAV_CMD_DO_SET_MODE: custom mode, PX4 main AUTO(4) sub LOITER(3)
        m.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3, 0, 0, 0, 0)
        print("sent DO_SET_MODE AUTO_LOITER", flush=True)
        t0 = time.time()
        while time.time() - t0 < 6:
            heartbeat()
            msg = m.recv_match(type=["COMMAND_ACK", "HEARTBEAT"],
                               blocking=True, timeout=0.5)
            if msg is not None and msg.get_type() == "COMMAND_ACK":
                print("ACK command=%s result=%s" % (msg.command, msg.result),
                      flush=True)
                break

    while True:
        heartbeat()
        while m.recv_match(blocking=False):
            pass
        time.sleep(1)


if __name__ == "__main__":
    main()
