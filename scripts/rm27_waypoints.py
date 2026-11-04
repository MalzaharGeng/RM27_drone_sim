#!/usr/bin/env python3
"""Fly a rectangular offboard waypoint path inside the RM27 arena.

Usage:
    python3 ~/drone_sim/scripts/rm27_waypoints.py

Requires PX4 SITL to be running (start_rm27_px4.sh). MAVSDK listens on
udp://:14540 and writes a position log to ~/drone_sim/logs/.
"""

import asyncio
import csv
import os
import time

from mavsdk_grpc import System
from mavsdk_grpc.offboard import OffboardError, PositionNedYaw
from mavsdk_grpc.telemetry import PositionNed

TAKEOFF_ALT_M = 2.0
# (north_m, east_m, down_m) relative to the arming position; down is negative up.
WAYPOINTS = [
    (0.0, 0.0, -TAKEOFF_ALT_M),
    (3.0, 0.0, -TAKEOFF_ALT_M),
    (3.0, 2.0, -TAKEOFF_ALT_M),
    (0.0, 2.0, -TAKEOFF_ALT_M),
    (0.0, 0.0, -TAKEOFF_ALT_M),
]
ARRIVE_TOLERANCE_M = 0.5
ARRIVE_TIMEOUT_S = 30


async def wait_until(predicate_coro, timeout_s, description):
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        if await predicate_coro():
            return
        await asyncio.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for {description}")


async def current_ned(drone):
    async for pos in drone.telemetry.position_velocity_ned():
        return pos.position
    return None


async def main():
    log_dir = os.path.expanduser("~/drone_sim/logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, "offboard_positions.csv")

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection ...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4")
            break

    print("Waiting for position estimate ...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_global_position_ok:
            print("Position estimate OK")
            break

    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.arm()
    print("Armed")
    await drone.action.takeoff()
    print(f"Taking off to {TAKEOFF_ALT_M} m")

    async def above_altitude():
        pos = await current_ned(drone)
        return pos is not None and pos.down_m <= -TAKEOFF_ALT_M + 0.3

    await wait_until(above_altitude, 30, "takeoff altitude")

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT_M, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        print(f"Offboard start failed: {exc}")
        await drone.action.land()
        return

    print("Offboard active")
    with open(log_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "north_m", "east_m", "down_m"])
        for index, (north, east, down) in enumerate(WAYPOINTS):
            print(f"Waypoint {index + 1}/{len(WAYPOINTS)}: N={north} E={east} D={down}")
            await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, 0.0))

            async def arrived():
                pos = await current_ned(drone)
                if pos is None:
                    return False
                dn = pos.north_m - north
                de = pos.east_m - east
                dd = pos.down_m - down
                writer.writerow([time.time(), pos.north_m, pos.east_m, pos.down_m])
                return (dn * dn + de * de + dd * dd) ** 0.5 < ARRIVE_TOLERANCE_M

            await wait_until(arrived, ARRIVE_TIMEOUT_S, f"waypoint {index + 1}")

    print(f"Waypoints complete; position log: {log_path}")
    await drone.offboard.stop()
    await drone.action.land()
    print("Landing")


if __name__ == "__main__":
    asyncio.run(main())
