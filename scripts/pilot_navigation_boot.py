#!/usr/bin/env python3
"""Start one pilot-authorized task from an explicit boot configuration."""
import argparse
import datetime
import json
import math
import os
import pathlib
import sys
import uuid


def arguments(config):
    if config.get("schema_version") != 1:
        raise ValueError("Expected schema_version=1")
    allowed = {"schema_version", "rc_selector", "rc_offboard_slot", "mission", "distance_m",
               "side_m", "speed_m_s", "max_radius_m", "max_alt_error_m", "max_tracking_error_m"}
    unknown = set(config)-allowed
    if unknown:
        raise ValueError("Unknown configuration keys: " + ", ".join(sorted(unknown)))
    if config.get("mission", "line") not in ("line", "square"):
        raise ValueError("mission must be line or square")
    args = ["--pilot-timeout", "0", "--mission", config.get("mission", "line")]
    for key, option, default, maximum in (
        ("distance_m", "--distance", 10., 30.), ("side_m", "--side", 1., 10.),
        ("speed_m_s", "--speed", .25, 1.), ("max_radius_m", "--max-radius", 11., 50.),
        ("max_alt_error_m", "--max-alt-error", .5, 2.),
        ("max_tracking_error_m", "--max-err", .8, 2.)):
        value = config.get(key, default)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError(f"{key} must be finite and in (0,{maximum}]")
        args.extend((option, str(value)))
    selector = config.get("rc_selector")
    if selector == "three_position":
        if config.get("rc_offboard_slot", 0) != 0:
            raise ValueError("three_position must not specify a fixed Offboard slot")
        args.append("--rc-three-position")
    elif selector == "dedicated":
        args.extend(("--rc-offboard-slot", "0"))
    elif selector == "fixed_slot":
        slot = config.get("rc_offboard_slot")
        if type(slot) is not int or slot not in range(1, 7):
            raise ValueError("fixed_slot requires rc_offboard_slot in 1..6")
        args.extend(("--rc-offboard-slot", str(slot)))
    else:
        raise ValueError("rc_selector must be three_position, dedicated or fixed_slot")
    return args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--check", action="store_true", help="validate/print arguments; starts no ROS node")
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise ValueError("Configuration must be an object")
        command = [sys.executable, str(pathlib.Path(__file__).with_name("px4_square_bt.py")), *arguments(config)]
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.check:
        print(json.dumps(dict(valid=True, command=command), ensure_ascii=False))
        return 0
    log_root = pathlib.Path(os.environ.get("PILOT_LOG_ROOT", "~/.local/state/pilot-navigation")).expanduser()
    log_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log = log_root/(stamp+"_"+uuid.uuid4().hex[:8]+".csv")
    command += ["--log", str(log)]
    print("Pilot boot: manual takeoff; selector="+config["rc_selector"]+"; log="+str(log), flush=True)
    os.execv(sys.executable, command)


if __name__ == "__main__":
    raise SystemExit(main())
