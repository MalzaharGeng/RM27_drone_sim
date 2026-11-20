#!/usr/bin/env python3
"""Unit tests for nav_planner.py.

Runs without ROS or MAVSDK:
    python3 ~/drone_sim/scripts/test_nav_planner.py
"""

from __future__ import annotations

import math
import pathlib
import unittest

from nav_planner import (
    PlanError,
    Rect,
    clearance,
    densify,
    load_world_map,
    path_length,
    plan,
    world_to_ned,
)

MARGIN = 0.8
HERE = pathlib.Path(__file__).resolve().parent


def sample_polyline(points, step=0.1):
    for start, end in zip(points, points[1:]):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(length / step)))
        for k in range(steps + 1):
            t = k / steps
            yield start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t


class EmptyMapTest(unittest.TestCase):
    def test_straight_line_when_nothing_blocks(self):
        path = plan((0.0, 0.0), (5.0, 0.0), [], margin=MARGIN)
        self.assertEqual(len(path), 2)
        self.assertAlmostEqual(path_length(path), 5.0, places=6)

    def test_clearance_is_infinite_without_obstacles(self):
        self.assertEqual(clearance(0.0, 0.0, []), math.inf)


class ObstacleTest(unittest.TestCase):
    def setUp(self):
        # 0.4 m thick wall straight between start and goal, 2 m wide.
        self.wall = Rect(2.0, -1.0, 2.4, 1.0)

    def test_path_detours_around_wall(self):
        start, goal = (0.0, 0.0), (5.0, 0.0)
        path = plan(start, goal, [self.wall], margin=MARGIN)
        straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
        self.assertGreater(path_length(path), straight + 0.5)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_path_keeps_margin(self):
        path = plan((0.0, 0.0), (5.0, 0.0), [self.wall], margin=MARGIN)
        worst = min(clearance(n, e, [self.wall]) for n, e in sample_polyline(path))
        self.assertGreaterEqual(worst, MARGIN - 0.05)

    def test_goal_inside_obstacle_raises(self):
        with self.assertRaises(PlanError):
            plan((0.0, 0.0), (2.2, 0.0), [self.wall], margin=MARGIN)

    def test_fully_walled_goal_raises(self):
        cage = [
            Rect(3.0, -3.0, 3.2, 3.0),
            Rect(3.0, 3.0, 8.0, 3.2),
            Rect(3.0, -3.2, 8.0, -3.0),
            Rect(7.8, -3.0, 8.0, 3.0),
        ]
        with self.assertRaises(PlanError):
            plan((0.0, 0.0), (5.5, 0.0), cage, margin=MARGIN)


class ArenaMapTest(unittest.TestCase):
    """The real RM27 obstacle map and mission targets must stay feasible."""

    @classmethod
    def setUpClass(cls):
        world_map = load_world_map(HERE / "obstacles_rm27.json")
        cls.obstacles = world_map.obstacles
        cls.targets = world_map.targets
        cls.spawn = world_map.spawn
        cls.landmarks = world_map.landmarks

    def test_world_is_enu(self):
        # Gazebo ENU -> PX4 NED swaps the axes: north is world y, east is x.
        self.assertEqual(world_to_ned(7.0, 12.0, (5.0, 6.0)), (6.0, 2.0))

    def test_arena_bounds_match_the_world_sdf(self):
        # Walls in rm27_world.sdf: x 0.15..28.15 (east), y 0.06..16.07 (north).
        west = min(r.east_min for r in self.obstacles)
        east = max(r.east_max for r in self.obstacles)
        south = min(r.north_min for r in self.obstacles)
        north = max(r.north_max for r in self.obstacles)
        self.assertAlmostEqual(west, 0.149 - 5.0, places=3)
        self.assertAlmostEqual(east, 28.149 - 5.0, places=3)
        self.assertAlmostEqual(south, 0.064 - 6.0, places=3)
        self.assertAlmostEqual(north, 16.067 - 6.0, places=3)

    def test_enemy_base_is_reachable(self):
        enemy = self.landmarks["base_far"]
        path = plan(self.spawn, enemy, self.obstacles, margin=MARGIN)
        self.assertEqual(path[-1], enemy)
        worst = min(
            clearance(n, e, self.obstacles) for n, e in sample_polyline(path)
        )
        self.assertGreaterEqual(worst, MARGIN - 0.05)

    def test_every_leg_is_plannable_and_safe(self):
        current = self.spawn
        detoured = 0
        for target in self.targets:
            path = plan(current, target, self.obstacles, margin=MARGIN)
            worst = min(clearance(n, e, self.obstacles) for n, e in sample_polyline(path))
            self.assertGreaterEqual(worst, MARGIN - 0.05, f"leg to {target} too close")
            straight = math.hypot(target[0] - current[0], target[1] - current[1])
            if path_length(path) > straight + 0.5:
                detoured += 1
            current = target
        # The map is only meaningful if most legs really have to go around.
        self.assertGreaterEqual(detoured, 2)

    def test_spawn_is_free(self):
        self.assertGreater(clearance(*self.spawn, self.obstacles), MARGIN)


class DensifyTest(unittest.TestCase):
    def test_no_hop_longer_than_step(self):
        path = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0)]
        dense = densify(path, 0.5)
        for start, end in zip(dense, dense[1:]):
            self.assertLessEqual(
                math.hypot(end[0] - start[0], end[1] - start[1]), 0.5 + 1e-9
            )
        self.assertEqual(dense[0], path[0])
        self.assertEqual(dense[-1], path[-1])
        self.assertEqual(len(dense), 1 + 20 + 10)

    def test_short_segment_is_kept(self):
        dense = densify([(0.0, 0.0), (0.2, 0.0)], 0.5)
        self.assertEqual(len(dense), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
