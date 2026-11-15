"""Verify legacy entrypoints cannot silently start the automatic flight path."""
import asyncio
import pathlib
import subprocess
import sys
import unittest
import rm27_nav
import rm27_waypoints

HERE = pathlib.Path(__file__).resolve().parent


class EntrypointTest(unittest.TestCase):
    def test_old_waypoint_api_requires_simulation_authorization(self):
        with self.assertRaisesRegex(RuntimeError, "simulation-only"):
            asyncio.run(rm27_waypoints.main())

    def test_old_waypoint_cli_rejects_before_connecting(self):
        result = subprocess.run([sys.executable, str(HERE/"rm27_waypoints.py")], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
        self.assertIn("simulation-only", result.stderr)

    def test_mavsdk_mission_requires_explicit_simulation_authorization(self):
        with self.assertRaisesRegex(RuntimeError, "simulation-only"):
            asyncio.run(rm27_nav.mission([], [], (0., 0.), 1., HERE/"unused.csv"))

    def test_default_navigation_cli_rejects_auto_flight(self):
        result = subprocess.run([sys.executable, str(HERE/"rm27_nav.py")], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
        self.assertIn("Live auto-flight is disabled", result.stderr)
        self.assertNotIn("connecting to PX4", result.stdout)

    def test_demo_launcher_rejects_before_starting_simulator(self):
        result = subprocess.run(["bash", str(HERE/"run_nav_demo.sh")], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
        self.assertNotIn("启动仿真", result.stdout)


if __name__ == "__main__":
    unittest.main()
