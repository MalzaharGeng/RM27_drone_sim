import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from pilot_navigation_boot import arguments


class BootConfigTest(unittest.TestCase):
    def test_three_position_boot_waits_without_timeout(self):
        args = arguments(dict(schema_version=1, rc_selector="three_position"))
        self.assertIn("--rc-three-position", args)
        self.assertEqual(args[args.index("--pilot-timeout")+1], "0")
        self.assertNotIn("--dry-run", args)

    def test_unknown_and_invalid_config_never_starts(self):
        for change in (dict(schema_version=2), dict(rc_selector="guess"), dict(speed_m_s=2.),
                       dict(distance_m=float("nan")), dict(max_radius_m=0), dict(arm=True),
                       dict(rc_offboard_slot=6)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                arguments({"schema_version": 1, "rc_selector": "three_position", **change})

    def test_fixed_slot_requires_explicit_valid_slot(self):
        self.assertIn("6", arguments(dict(schema_version=1, rc_selector="fixed_slot", rc_offboard_slot=6)))
        with self.assertRaises(ValueError):
            arguments(dict(schema_version=1, rc_selector="fixed_slot"))

    def test_check_mode_creates_no_state_or_process(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory)/"config.json"
            path.write_text(json.dumps(dict(schema_version=1, rc_selector="three_position")))
            proc = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name("pilot_navigation_boot.py")),
                                   "--config", str(path), "--check"], capture_output=True, text=True, timeout=5)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertTrue(json.loads(proc.stdout)["valid"])
            self.assertEqual([p.name for p in pathlib.Path(directory).iterdir()], ["config.json"])


if __name__ == "__main__":
    unittest.main()
