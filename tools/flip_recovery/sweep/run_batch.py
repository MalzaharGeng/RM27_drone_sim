"""Run a declared matrix with bounded parallelism and isolated instances."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("manifest", type=Path)
    p.add_argument("repo", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--jobs", type=int, default=3)
    p.add_argument("--first-instance", type=int, default=21)
    args = p.parse_args()
    cases = json.loads(args.manifest.read_text())
    args.out.mkdir(parents=True)
    (args.out/"matrix.json").write_text(json.dumps(cases, indent=2)+"\n")
    script = Path(__file__).with_name("run_case.py")

    def lane(item):
        slot, work = item
        for case in work:
            cmd = [sys.executable, str(script), "--repo", str(args.repo),
                   "--out", str(args.out/case["id"]), "--instance", str(args.first_instance+slot),
                   "--params", json.dumps(case.get("params", {}))]
            for key in ("tilt","axis","yaw","height","model","world","airframe","entity","startup_timeout","cancel_after"):
                if key in case:
                    cmd += ["--"+key.replace("_","-"), str(case[key])]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            meta = args.out/case["id"]/"meta.json"
            status = json.loads(meta.read_text()).get("error", "recorded") if meta.exists() else proc.stderr
            print(case["id"], proc.returncode, status, flush=True)

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        list(pool.map(lane, enumerate([cases[i::args.jobs] for i in range(args.jobs)])))


if __name__ == "__main__":
    main()
