#!/usr/bin/env python3
"""Waypoint navigation with static obstacle avoidance inside the RM27 arena.

Each leg is planned with A* around the known arena geometry (see
obstacles_rm27.json), then flown as position setpoints. Positions and the
live distance to the nearest obstacle are logged to logs/nav_positions.csv,
and the flight aborts and lands if the drone gets closer than
MIN_CLEARANCE_M to an obstacle.

Usage:
    python3 ~/drone_sim/scripts/rm27_nav.py --dry-run        # plan only, no sim
    python3 ~/drone_sim/scripts/rm27_nav.py                  # fly the mission
    python3 ~/drone_sim/scripts/rm27_nav.py --enemy-base     # straight to far base

Requires PX4 SITL started with the nav world:
    PX4_GZ_WORLD=rm27_nav_world ~/drone_sim/start_rm27_px4.sh
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import pathlib
import sys
import time

from nav_planner import (
    PlanError,
    clearance,
    densify,
    load_world_map,
    path_length,
    plan,
    world_to_ned,
)

try:  # the planner and --dry-run work without MAVSDK installed
    from mavsdk_grpc import System
    from mavsdk_grpc.action import ActionError
    from mavsdk_grpc.offboard import OffboardError, PositionNedYaw

    HAVE_MAVSDK = True
except ImportError:  # pragma: no cover - exercised only on machines without MAVSDK
    HAVE_MAVSDK = False

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_MAP = HERE / "obstacles_rm27.json"
DEFAULT_LOG = HERE.parent / "logs" / "nav_positions.csv"

MARGIN_M = 1.2
SPEED_HINT_MPS = 1.0
STEP_M = 0.5
ARRIVE_TOLERANCE_M = 0.3
MIN_CLEARANCE_M = 0.6
ALTITUDE_GUARD_M = 0.8
LEG_TIMEOUT_S = 120.0
CONNECT_TIMEOUT_S = 30.0
POSITION_TIMEOUT_S = 45.0
ARM_TIMEOUT_S = 30.0
ARM_RETRIES = 15
ARM_RETRY_PERIOD_S = 2.0
SETPOINT_PERIOD_S = 0.2
FINAL_HOVER_ALT_M = 0.8
DESCENT_RATE_MPS = 0.4
DESCENT_TOLERANCE_M = 0.15
DESCENT_TIMEOUT_S = 40.0


class SafetyAbort(RuntimeError):
    """Raised when the drone gets too close to a known obstacle."""


def plan_mission(obstacles, targets, spawn):
    """Plan every leg up front so --dry-run can show the whole route."""
    legs = []
    current = spawn
    for target in targets:
        path = plan(current, target, obstacles, margin=MARGIN_M)
        legs.append((current, target, path))
        current = target
    return legs


def describe_mission(legs, obstacles):
    total = 0.0
    for index, (start, target, path) in enumerate(legs, 1):
        straight = math.hypot(target[0] - start[0], target[1] - start[1])
        length = path_length(path)
        total += length
        worst = min(
            clearance(n, e, obstacles) for n, e in path[1:-1]
        ) if len(path) > 2 else clearance(*path[0], obstacles)
        print(
            f"leg {index}: N={target[0]:+.1f} E={target[1]:+.1f}  "
            f"heading {_heading_deg(start, target):+.0f} deg  "
            f"直飞 {straight:.2f} m / 规划 {length:.2f} m  "
            f"拐点 {len(path) - 2}  最窄余量 {worst:.2f} m"
        )
        for point in path[1:-1]:
            print(f"    via N={point[0]:+.2f} E={point[1]:+.2f}")
    print(f"total planned distance: {total:.2f} m")


async def current_sample(drone):
    async for sample in drone.telemetry.position_velocity_ned():
        return sample
    return None


async def current_ned(drone):
    sample = await current_sample(drone)
    return sample.position if sample is not None else None


def _distance(point, north, east):
    return math.hypot(point[0] - north, point[1] - east)


def _heading_deg(start, end):
    """NED heading in degrees (0 = north, 90 = east) from ``start`` to ``end``."""
    north = end[0] - start[0]
    east = end[1] - start[1]
    if abs(north) + abs(east) < 1e-6:
        return 0.0
    return math.degrees(math.atan2(east, north))


def _velocity(sample):
    """Return (horizontal speed, vertical speed) in m/s, or nans if absent."""
    velocity = getattr(sample, "velocity", None)
    if velocity is None:
        return float("nan"), float("nan")
    north = getattr(velocity, "north_m_s", float("nan"))
    east = getattr(velocity, "east_m_s", float("nan"))
    down = getattr(velocity, "down_m_s", float("nan"))
    return math.hypot(north, east), down


async def fly_to(drone, point, down_m, obstacles, writer, leg_index, yaw_deg):
    """Stream setpoints towards ``point`` until the drone is close enough.

    ``yaw_deg`` points the nose at the leg's final target, so the drone keeps
    facing the base instead of always flying north-facing (i.e. sideways).
    """
    started = time.monotonic()
    while True:
        sample = await current_sample(drone)
        if sample is None:
            raise RuntimeError("no position estimate from PX4")
        position = sample.position
        speed, vz = _velocity(sample)
        clear = clearance(position.north_m, position.east_m, obstacles)
        writer.writerow(
            [
                round(time.time(), 3),
                round(position.north_m, 3),
                round(position.east_m, 3),
                round(position.down_m, 3),
                round(speed, 2),
                round(vz, 2),
                round(clear, 3),
                leg_index,
                f"{point[0]:.2f},{point[1]:.2f}",
            ]
        )
        if clear < MIN_CLEARANCE_M:
            raise SafetyAbort(
                f"clearance {clear:.2f} m below {MIN_CLEARANCE_M} m near "
                f"N={position.north_m:.2f} E={position.east_m:.2f}"
            )
        if abs(position.down_m - down_m) > ALTITUDE_GUARD_M:
            raise SafetyAbort(
                f"altitude {-position.down_m:.2f} m drifted from {-down_m:.2f} m"
            )
        if _distance(point, position.north_m, position.east_m) < ARRIVE_TOLERANCE_M:
            return
        if time.monotonic() - started > LEG_TIMEOUT_S:
            raise TimeoutError(f"timed out flying to N={point[0]:.2f} E={point[1]:.2f}")
        await drone.offboard.set_position_ned(
            PositionNedYaw(point[0], point[1], down_m, yaw_deg)
        )
        await asyncio.sleep(SETPOINT_PERIOD_S)


async def descend_to(drone, north, east, down_m, obstacles, writer, leg_index, yaw_deg):
    """Lower the drone to ``down_m`` while holding the target horizontal point.

    Done as a ramped stream of setpoints so the descent stays slower than
    PX4's default and no altitude guard trips on the intentional drop.
    """
    started = time.monotonic()
    while True:
        sample = await current_sample(drone)
        if sample is None:
            raise RuntimeError("no position estimate from PX4")
        position = sample.position
        speed, vz = _velocity(sample)
        clear = clearance(position.north_m, position.east_m, obstacles)
        writer.writerow(
            [
                round(time.time(), 3),
                round(position.north_m, 3),
                round(position.east_m, 3),
                round(position.down_m, 3),
                round(speed, 2),
                round(vz, 2),
                round(clear, 3),
                leg_index,
                f"descend,{north:.2f},{east:.2f}",
            ]
        )
        if clear < MIN_CLEARANCE_M:
            raise SafetyAbort(
                f"clearance {clear:.2f} m below {MIN_CLEARANCE_M} m near "
                f"N={position.north_m:.2f} E={position.east_m:.2f}"
            )
        error = position.down_m - down_m
        if abs(error) <= DESCENT_TOLERANCE_M:
            return
        if time.monotonic() - started > DESCENT_TIMEOUT_S:
            raise TimeoutError(f"timed out descending to {-down_m:.2f} m")
        step = DESCENT_RATE_MPS * SETPOINT_PERIOD_S
        fraction = min(1.0, step / abs(error))
        target_down = position.down_m + (down_m - position.down_m) * fraction
        await drone.offboard.set_position_ned(
            PositionNedYaw(north, east, target_down, yaw_deg)
        )
        await asyncio.sleep(SETPOINT_PERIOD_S)


async def hover(drone, seconds, down_m, writer, leg_index, yaw_deg):
    """Hold the current position, keeping the offboard setpoint stream alive."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        sample = await current_sample(drone)
        if sample is None:
            return
        position = sample.position
        speed, vz = _velocity(sample)
        writer.writerow(
            [
                round(time.time(), 3),
                round(position.north_m, 3),
                round(position.east_m, 3),
                round(position.down_m, 3),
                round(speed, 2),
                round(vz, 2),
                "",
                leg_index,
                "hold",
            ]
        )
        await drone.offboard.set_position_ned(
            PositionNedYaw(position.north_m, position.east_m, down_m, yaw_deg)
        )
        await asyncio.sleep(SETPOINT_PERIOD_S)


async def watch_flight_mode(drone):
    """Print every PX4 flight mode change, so a surprise landing is explainable."""
    last = None
    try:
        async for mode in drone.telemetry.flight_mode():
            name = getattr(mode, "name", str(mode))
            if name != last:
                print(f"[px4 mode] {name}", flush=True)
                last = name
    except Exception as exc:  # noqa: BLE001 - diagnostics must never kill the mission
        print(f"[px4 mode] watcher stopped: {exc}", file=sys.stderr)


async def _connect_and_wait():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            return drone
    raise RuntimeError("PX4 connection stream closed")


async def _wait_position_ready(drone):
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_global_position_ok:
            return


async def _wait_armable(drone):
    """PX4 reports ``is_armable`` only after every preflight check passes.

    Right after the simulator starts the EKF/GCS checks are still settling,
    so arm() returns COMMAND_DENIED. Wait for health first instead of racing.
    """
    async for health in drone.telemetry.health():
        if health.is_armable:
            return


async def _arm_with_retry(drone):
    last_error = None
    for attempt in range(1, ARM_RETRIES + 1):
        try:
            await drone.action.arm()
            return
        except ActionError as exc:
            last_error = exc
            print(
                f"arm denied ({attempt}/{ARM_RETRIES}): {exc}"
                f" - retrying in {ARM_RETRY_PERIOD_S:.0f} s",
                flush=True,
            )
            await asyncio.sleep(ARM_RETRY_PERIOD_S)
    raise RuntimeError(f"could not arm after {ARM_RETRIES} attempts: {last_error}")


async def mission(
    obstacles,
    targets,
    spawn,
    altitude_m,
    log_path,
    hold_s=0.0,
    final_hover_alt_m=FINAL_HOVER_ALT_M,
):
    print("connecting to PX4 on udp 14540 (Ctrl-C to abort) ...")
    try:
        drone = await asyncio.wait_for(_connect_and_wait(), CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(
            f"no PX4 after {CONNECT_TIMEOUT_S:.0f} s on udp 14540; is the simulator "
            "running? check with: ss -lunp | grep 14540",
            file=sys.stderr,
        )
        return 1
    print("connected to PX4")
    asyncio.create_task(watch_flight_mode(drone))

    print("waiting for position estimate ...")
    try:
        await asyncio.wait_for(_wait_position_ready(drone), POSITION_TIMEOUT_S)
    except asyncio.TimeoutError:
        print("no position estimate; check the PX4 console for EKF errors", file=sys.stderr)
        return 1
    print("position estimate OK")

    print("waiting for preflight checks ...")
    try:
        await asyncio.wait_for(_wait_armable(drone), ARM_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(
            "PX4 never reported armable; check the PX4 console for preflight "
            "failures (EKF, sensors, GCS connection)",
            file=sys.stderr,
        )
        return 1
    print("preflight checks OK")

    # Dense waypoints already cap the speed, but ask PX4 for a low limit too.
    # Older/newer mavsdk builds may not expose the param plugin, so this is
    # best effort and must never abort the mission.
    for name, value in (
        ("MPC_XY_VEL_MAX", 2.0),
        ("MPC_Z_VEL_MAX_UP", 1.0),
        ("MPC_Z_VEL_MAX_DN", 0.7),
        # 没有遥控器时 offboard 一丢就自动降落，给 setpoint 流留点余量。
        ("COM_OF_LOSS_T", 3.0),
    ):
        try:
            await drone.param.set_param_float(name, value)
        except Exception as exc:  # noqa: BLE001 - API differs between mavsdk builds
            print(f"could not set {name}: {exc}", file=sys.stderr)
    try:
        # COM_DL_LOSS_T is an int32 in PX4; sending it as float is rejected.
        await drone.param.set_param_int("COM_DL_LOSS_T", 30)
    except Exception as exc:  # noqa: BLE001 - API differs between mavsdk builds
        print(f"could not set COM_DL_LOSS_T: {exc}", file=sys.stderr)

    down_m = -altitude_m
    await drone.action.set_takeoff_altitude(altitude_m)
    await _arm_with_retry(drone)
    await drone.action.takeoff()
    print(f"taking off to {altitude_m:.1f} m")

    takeoff_started = time.monotonic()
    while True:
        position = await current_ned(drone)
        if position is not None and position.down_m <= down_m + 0.3:
            break
        if time.monotonic() - takeoff_started > 60:
            print("takeoff did not reach altitude; landing", file=sys.stderr)
            await drone.action.land()
            return 1
        await asyncio.sleep(0.5)

    await drone.offboard.set_position_ned(
        PositionNedYaw(position.north_m, position.east_m, down_m, 0.0)
    )
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        print(f"offboard start failed: {exc}")
        await drone.action.land()
        return 1

    print("offboard active")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    legs = plan_mission(obstacles, targets, spawn)
    try:
        with log_path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "t",
                    "north_m",
                    "east_m",
                    "down_m",
                    "speed_m_s",
                    "vz_m_s",
                    "clearance_m",
                    "leg",
                    "setpoint",
                ]
            )
            last_yaw = 0.0
            for index, (start, target, path) in enumerate(legs, 1):
                hops = densify(path, STEP_M)
                last_yaw = _heading_deg(start, target)
                print(
                    f"leg {index}/{len(legs)} -> N={target[0]:+.1f} E={target[1]:+.1f} "
                    f"({len(hops) - 1} setpoints, {STEP_M} m apart, "
                    f"heading {last_yaw:+.0f} deg)"
                )
                for point in hops[1:]:
                    await fly_to(
                        drone, point, down_m, obstacles, writer, index, last_yaw
                    )
            final_altitude = altitude_m
            if 0 < final_hover_alt_m < altitude_m:
                final_altitude = final_hover_alt_m
            final_down = -final_altitude
            if final_altitude < altitude_m:
                target = targets[-1]
                print(
                    f"descending to {final_altitude:.1f} m in front of the "
                    f"target, then holding {hold_s:.0f} s"
                )
                await descend_to(
                    drone,
                    target[0],
                    target[1],
                    final_down,
                    obstacles,
                    writer,
                    len(legs),
                    last_yaw,
                )
            if hold_s > 0:
                print(f"holding {hold_s:.0f} s at the last target")
                await hover(drone, hold_s, final_down, writer, len(legs), last_yaw)
    except SafetyAbort as exc:
        print(f"safety abort: {exc}")
        await drone.offboard.stop()
        await drone.action.land()
        return 2
    except (PlanError, TimeoutError, RuntimeError) as exc:
        print(f"mission failed: {exc}")
        await drone.offboard.stop()
        await drone.action.land()
        return 1

    print(f"mission complete, log: {log_path}")
    await drone.offboard.stop()
    await drone.action.land()
    return 0


def _parse_world_xy(text: str) -> tuple[float, float]:
    try:
        x_text, y_text = text.replace(" ", "").split(",")
        return float(x_text), float(y_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"expected world coordinates as x,y (got {text!r})"
        ) from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=pathlib.Path, default=DEFAULT_MAP)
    parser.add_argument("--log", type=pathlib.Path, default=DEFAULT_LOG)
    parser.add_argument("--altitude", type=float, default=None)
    parser.add_argument(
        "--target",
        type=_parse_world_xy,
        action="append",
        metavar="X,Y",
        help="fly to this Gazebo world coordinate (x=east, y=north); "
        "repeat for several targets. Overrides targets_world_xy in the map.",
    )
    bases = parser.add_mutually_exclusive_group()
    bases.add_argument(
        "--enemy-base",
        action="store_true",
        help="fly to the far (enemy) base, landing just in front of its armour plate",
    )
    bases.add_argument(
        "--own-base",
        action="store_true",
        help="fly to the near (own) base from landmarks_world_xy",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=3.0,
        help="seconds to hover at the last target before landing (default 3)",
    )
    parser.add_argument(
        "--final-hover-altitude",
        type=float,
        default=FINAL_HOVER_ALT_M,
        help="altitude to descend to for the final hover before landing "
        f"(default {FINAL_HOVER_ALT_M} m; use 0 to skip the descent)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the planned route and exit without touching the simulator",
    )
    args = parser.parse_args()

    world_map = load_world_map(args.map)
    obstacles = world_map.obstacles
    targets = world_map.targets
    spawn = world_map.spawn
    altitude_m = world_map.altitude_m

    if args.enemy_base or args.own_base:
        landmark = "base_far" if args.enemy_base else "base_near"
        # The far base itself is occupied by the 1.7 m armour plate; land just
        # in front of it instead of hovering over the plate. The plate front
        # face is at world x=25.52, so base_far_front sits ~0.8 m clear of it.
        if args.enemy_base and "base_far_front" in world_map.landmarks:
            landmark = "base_far_front"
        if landmark not in world_map.landmarks:
            print(
                f"{args.map} has no landmark '{landmark}'",
                file=sys.stderr,
            )
            return 1
        targets = [world_map.landmarks[landmark]]
        print(
            f"target: {'enemy' if args.enemy_base else 'own'} base "
            f"({landmark})"
        )
    if args.target:
        # --target 给的是 Gazebo ENU 世界坐标，换成 PX4 本地 NED 再规划。
        targets = [
            world_to_ned(x, y, world_map.spawn_world_xy) for x, y in args.target
        ]
    if args.altitude is not None:
        altitude_m = args.altitude

    try:
        legs = plan_mission(obstacles, targets, spawn)
    except PlanError as exc:
        print(f"planning failed: {exc}", file=sys.stderr)
        return 1

    describe_mission(legs, obstacles)
    if args.dry_run:
        print("dry run: nothing was flown")
        return 0

    if not HAVE_MAVSDK:
        print(
            "mavsdk_grpc is not installed; pip install mavsdk-grpc "
            "or use --dry-run",
            file=sys.stderr,
        )
        return 1

    try:
        return asyncio.run(
            mission(
                obstacles,
                targets,
                spawn,
                altitude_m,
                args.log,
                hold_s=args.hold,
                final_hover_alt_m=args.final_hover_altitude,
            )
        )
    except (ActionError, OffboardError, RuntimeError) as exc:
        print(f"mission failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
