#!/usr/bin/env python3
"""Static obstacle path planning for the RM27 arena.

Pure standard library, no ROS/MAVSDK dependency, so the planner can be
unit tested outside the simulator (see test_nav_planner.py).

Coordinates are local NED in metres: (north, east). Obstacles are axis
aligned rectangles, which is what the arena walls and the nav blocks are.

Usage:
    from nav_planner import Rect, plan, clearance
    path = plan((0.0, 0.0), (13.0, -3.0), [Rect(3, -2, 5, 0)])
"""

from __future__ import annotations

import heapq
import json
import math
import pathlib
from dataclasses import dataclass, field

SQRT2 = math.sqrt(2.0)
Point = tuple[float, float]


def world_to_ned(x: float, y: float, spawn_world_xy: tuple[float, float]) -> Point:
    """Convert Gazebo world coordinates to PX4 local NED.

    Gazebo simulates in an ENU world (x = east, y = north), and PX4's
    gz_bridge turns that into NED by swapping x/y (see GZBridge.cpp:
    ``(north, east) = (gz.y, gz.x)``). ``spawn_world_xy`` is the vehicle's
    spawn point in the same ENU frame.
    """
    spawn_east, spawn_north = spawn_world_xy
    return (y - spawn_north, x - spawn_east)


class PlanError(RuntimeError):
    """Raised when no collision free path can be found."""


@dataclass(frozen=True)
class Rect:
    """Axis aligned obstacle rectangle in the (north, east) plane."""

    north_min: float
    east_min: float
    north_max: float
    east_max: float

    def inflated(self, margin: float) -> "Rect":
        return Rect(
            self.north_min - margin,
            self.east_min - margin,
            self.north_max + margin,
            self.east_max + margin,
        )

    def contains(self, north: float, east: float) -> bool:
        return (
            self.north_min <= north <= self.north_max
            and self.east_min <= east <= self.east_max
        )

    def distance(self, north: float, east: float) -> float:
        """Shortest distance from a point to the rectangle; 0 inside it."""
        dn = max(self.north_min - north, 0.0, north - self.north_max)
        de = max(self.east_min - east, 0.0, east - self.east_max)
        return math.hypot(dn, de)


@dataclass(frozen=True)
class WorldMap:
    """Arena map expressed in local NED, ready for the planner."""

    obstacles: list[Rect]
    targets: list[Point]
    spawn: Point = (0.0, 0.0)
    altitude_m: float = 2.0
    spawn_world_xy: tuple[float, float] = (0.0, 0.0)
    landmarks: dict[str, Point] = field(default_factory=dict)


def load_world_map(path: str | pathlib.Path) -> WorldMap:
    """Load an obstacles_*.json file, converting ENU world -> local NED."""
    data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    spawn_east, spawn_north = (float(v) for v in data["spawn_world_xy"])
    origin = (spawn_east, spawn_north)

    obstacles: list[Rect] = []
    for entry in data["obstacles"]:
        x0, x1 = entry["x"]
        y0, y1 = entry["y"]
        n0, e0 = world_to_ned(x0, y0, origin)
        n1, e1 = world_to_ned(x1, y1, origin)
        obstacles.append(Rect(min(n0, n1), min(e0, e1), max(n0, n1), max(e0, e1)))

    targets = [world_to_ned(x, y, origin) for x, y in data["targets_world_xy"]]
    landmarks: dict[str, Point] = {}
    for name, value in data.get("landmarks_world_xy", {}).items():
        if name == "note":
            continue
        x, y = value
        landmarks[name] = world_to_ned(x, y, origin)
    return WorldMap(
        obstacles=obstacles,
        targets=targets,
        altitude_m=float(data.get("altitude_m", 2.0)),
        spawn_world_xy=origin,
        landmarks=landmarks,
    )


def clearance(north: float, east: float, obstacles: list[Rect]) -> float:
    """Distance to the nearest obstacle; ``inf`` when the map is empty."""
    return min((rect.distance(north, east) for rect in obstacles), default=math.inf)


def path_length(points: list[Point]) -> float:
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])
    )


def densify(points: list[Point], step: float = 0.5) -> list[Point]:
    """Insert intermediate points so no segment is longer than ``step``.

    PX4 flies a position setpoint at full speed, so handing it a setpoint
    11 m away means 7 m/s and a violent overshoot. Short hops keep the
    position error small and the flight smooth.
    """
    if step <= 0:
        raise ValueError("step must be positive")
    dense = [points[0]]
    for start, end in zip(points, points[1:]):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        count = max(1, int(math.ceil(length / step)))
        for index in range(1, count + 1):
            t = index / count
            dense.append(
                (start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t)
            )
    return dense


def _segment_free(
    start: Point, end: Point, inflated: list[Rect], cell: float
) -> bool:
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(2, int(math.ceil(length / (cell * 0.5))))
    for step in range(steps + 1):
        t = step / steps
        north = start[0] + (end[0] - start[0]) * t
        east = start[1] + (end[1] - start[1]) * t
        if any(rect.contains(north, east) for rect in inflated):
            return False
    return True


def _simplify(points: list[Point], inflated: list[Rect], cell: float) -> list[Point]:
    """Greedy line-of-sight shortcutting: drop every redundant grid corner."""
    if len(points) <= 2:
        return list(points)
    simplified = [points[0]]
    index = 0
    while index < len(points) - 1:
        far = len(points) - 1
        while far > index + 1 and not _segment_free(
            points[index], points[far], inflated, cell
        ):
            far -= 1
        simplified.append(points[far])
        index = far
    return simplified


def plan(
    start: Point,
    goal: Point,
    obstacles: list[Rect],
    *,
    cell: float = 0.2,
    margin: float = 0.8,
    bounds: tuple[float, float, float, float] | None = None,
) -> list[Point]:
    """Plan a collision free path from ``start`` to ``goal``.

    ``margin`` is the obstacle inflation radius: keep it larger than the
    vehicle radius plus tracking error. The returned polyline starts at
    ``start`` and ends at ``goal``; intermediate points are shortcut corners.
    """
    inflated = [rect.inflated(margin) for rect in obstacles]

    if any(rect.contains(*goal) for rect in inflated):
        raise PlanError("goal is inside an inflated obstacle")
    if any(rect.contains(*start) for rect in inflated):
        raise PlanError("start is inside an inflated obstacle")

    if bounds is None:
        north_values = [start[0], goal[0]]
        east_values = [start[1], goal[1]]
        for rect in inflated:
            north_values += [rect.north_min, rect.north_max]
            east_values += [rect.east_min, rect.east_max]
        pad = 2.0
        bounds = (
            min(north_values) - pad,
            min(east_values) - pad,
            max(north_values) + pad,
            max(east_values) + pad,
        )

    north_min, east_min, north_max, east_max = bounds
    nx = int(math.floor((north_max - north_min) / cell)) + 1
    ny = int(math.floor((east_max - east_min) / cell)) + 1

    def to_index(point: Point) -> tuple[int, int]:
        i = int(round((point[0] - north_min) / cell))
        j = int(round((point[1] - east_min) / cell))
        return min(max(i, 0), nx - 1), min(max(j, 0), ny - 1)

    def to_point(i: int, j: int) -> Point:
        return north_min + i * cell, east_min + j * cell

    def is_blocked(i: int, j: int) -> bool:
        north, east = to_point(i, j)
        return any(rect.contains(north, east) for rect in inflated)

    start_index = to_index(start)
    goal_index = to_index(goal)

    def nearest_free(index: tuple[int, int]) -> tuple[int, int]:
        i0, j0 = index
        if not is_blocked(i0, j0):
            return index
        for radius in range(1, 40):
            for di in range(-radius, radius + 1):
                for dj in range(-radius, radius + 1):
                    if max(abs(di), abs(dj)) != radius:
                        continue
                    i, j = i0 + di, j0 + dj
                    if 0 <= i < nx and 0 <= j < ny and not is_blocked(i, j):
                        return i, j
        raise PlanError("no free cell near the requested point")

    start_index = nearest_free(start_index)
    goal_index = nearest_free(goal_index)

    def heuristic(i: int, j: int) -> float:
        di = abs(i - goal_index[0])
        dj = abs(j - goal_index[1])
        return (di + dj) + (SQRT2 - 2.0) * min(di, dj)

    neighbours = (
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, SQRT2),
        (1, -1, SQRT2),
        (-1, 1, SQRT2),
        (-1, -1, SQRT2),
    )

    open_heap: list[tuple[float, tuple[int, int]]] = [(heuristic(*start_index), start_index)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start_index: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal_index:
            break
        i, j = current
        base = cost[current]
        for di, dj, step in neighbours:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            if is_blocked(ni, nj):
                continue
            if di and dj and (is_blocked(i + di, j) or is_blocked(i, j + dj)):
                continue  # no corner cutting
            candidate = base + step
            if candidate < cost.get((ni, nj), math.inf):
                cost[(ni, nj)] = candidate
                came_from[(ni, nj)] = current
                heapq.heappush(open_heap, (candidate + heuristic(ni, nj), (ni, nj)))
    else:
        raise PlanError("no path found")

    if goal_index not in cost:
        raise PlanError("no path found")

    cells = [goal_index]
    while cells[-1] != start_index:
        cells.append(came_from[cells[-1]])
    cells.reverse()

    if len(cells) == 1:
        return [start, goal]

    points = [to_point(i, j) for i, j in cells]
    points[0] = start
    points[-1] = goal
    return _simplify(points, inflated, cell)
