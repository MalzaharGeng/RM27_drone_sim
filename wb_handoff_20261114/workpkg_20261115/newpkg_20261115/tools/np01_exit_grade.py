#!/usr/bin/env python3
"""NP-01 exit grading (A/B) for a completed run directory.

Read-only post-processing. It never touches the pilot, the Gate, check_run.py,
or any control path, and it never rewrites run evidence.

Grade contract
--------------
A  graceful exit
   - the pilot reached LANDED/DONE and the final sample shows armed == False
     (disarm confirmed), and
   - the independent truth watchdog did not trigger, and
   - no TRUTH_BOUND_EXCEEDED marker is present.
   Subtypes:
     A_complete : task_completed == True  (nominal)
     A_early    : task_completed == False (pre-emptive cancellation, still landed)

B  emergency termination / uncontrolled
   - the truth watchdog triggered, or TRUTH_BOUND_EXCEEDED exists, or
   - the run ended with landing NOT confirmed (armed still True, or the pilot
     never reached LANDED/DONE).
   A grade B run MUST NOT be reported as a safe exit, and MUST be recorded as
   an emergency termination with an unconfirmed landing.

Usage:
    python3 np01_exit_grade.py <run_dir> [--write]

Without --write it only prints. With --write it emits <run_dir>/exit_grade.json.
"""

import argparse
import csv
import json
import os
import sys

DONE_STATES = {"LANDED", "DONE"}


def _last_row(path):
    last = None
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            last = row
    return last


def _truthy(v):
    return str(v).strip().lower() in ("true", "1", "yes")


def classify(run_dir):
    pilot_path = os.path.join(run_dir, "pilot.csv")
    reasons = []
    evidence = {}

    if not os.path.exists(pilot_path):
        return {
            "grade": "B",
            "subtype": "B_unknown",
            "landing_confirmed": False,
            "safe_exit_claim_allowed": False,
            "reasons": ["pilot.csv missing; landing cannot be confirmed"],
            "evidence": {},
        }

    last = _last_row(pilot_path) or {}
    final_state = (last.get("state") or "").strip()
    final_armed = _truthy(last.get("armed"))
    task_completed = _truthy(last.get("task_completed"))
    stop_reason = (last.get("ex_stop_reason") or "").strip()

    evidence["pilot_final_state"] = final_state
    evidence["pilot_final_armed"] = final_armed
    evidence["pilot_task_completed"] = task_completed
    evidence["pilot_stop_reason"] = stop_reason

    reached_landing = final_state in DONE_STATES
    landing_confirmed = reached_landing and not final_armed

    wd_path = os.path.join(run_dir, "truth_watchdog.json")
    wd_triggered = False
    if os.path.exists(wd_path):
        try:
            wd = json.load(open(wd_path))
            wd_triggered = bool(wd.get("triggered"))
            if wd_triggered:
                ev = wd.get("event") or {}
                evidence["watchdog_reason"] = ev.get("reason")
                evidence["watchdog_disp_xy_m"] = ev.get("disp_xy_m")
                evidence["watchdog_angular_rate_deg_s"] = ev.get("angular_rate_deg_s")
        except Exception as exc:  # noqa: BLE001 - evidence only
            evidence["watchdog_parse_error"] = str(exc)

    marker = os.path.join(run_dir, "TRUTH_BOUND_EXCEEDED")
    marker_exists = os.path.exists(marker)
    evidence["truth_bound_marker"] = marker_exists

    if wd_triggered:
        reasons.append("independent truth watchdog triggered")
    if marker_exists:
        reasons.append("TRUTH_BOUND_EXCEEDED marker present")
    if not reached_landing:
        reasons.append("pilot never reached LANDED/DONE (final state %r)" % final_state)
    if final_armed:
        reasons.append("final sample still armed: landing/disarm not confirmed")

    if wd_triggered or marker_exists or not landing_confirmed:
        grade = "B"
        if wd_triggered or marker_exists:
            subtype = "B_emergency_terminated"
        else:
            subtype = "B_landing_unconfirmed"
    else:
        grade = "A"
        subtype = "A_complete" if task_completed else "A_early"

    return {
        "grade": grade,
        "subtype": subtype,
        "landing_confirmed": landing_confirmed,
        "safe_exit_claim_allowed": grade == "A",
        "must_not_report_as_safe_exit": grade == "B",
        "reasons": reasons,
        "evidence": evidence,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="NP-01 exit grading (A/B), read-only")
    ap.add_argument("run_dir")
    ap.add_argument("--write", action="store_true", help="emit exit_grade.json")
    a = ap.parse_args(argv)

    if not os.path.isdir(a.run_dir):
        print("not a directory: %s" % a.run_dir, file=sys.stderr)
        return 2

    result = classify(a.run_dir)
    result["run_dir"] = os.path.abspath(a.run_dir)
    result["note"] = (
        "Post-processing only. Truth is used for grading, never for control. "
        "Grade B must be reported as an emergency termination with an "
        "unconfirmed landing, never as a safe exit."
    )

    print(json.dumps(result, indent=2, sort_keys=True))

    if a.write:
        out = os.path.join(a.run_dir, "exit_grade.json")
        if os.path.exists(out):
            print("refusing to overwrite existing %s" % out, file=sys.stderr)
            return 3
        with open(out, "w") as f:
            json.dump(result, f, indent=2, sort_keys=True)
            f.write("\n")
        print("wrote %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
