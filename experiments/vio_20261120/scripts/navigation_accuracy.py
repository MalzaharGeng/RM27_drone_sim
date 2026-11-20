#!/usr/bin/env python3
"""Report navigation accuracy against independent, pre-aligned ground truth.

CSV: t_s,truth_t_s,vio_n,vio_e,vio_d,truth_n,truth_e,truth_d
Optional: valid (0/1), reference_epoch, target_n,target_e,target_d.
No trajectory fitting, bias removal or scale correction is performed.
"""
import argparse
import csv
import json
import math
import pathlib


def percentile(values, fraction):
    values = sorted(values)
    x = (len(values)-1)*fraction
    lo, hi = math.floor(x), math.ceil(x)
    return values[lo] + (values[hi]-values[lo])*(x-lo)


def summarize(path, max_pair_skew=.05, max_gap=.25, jump_threshold=.5):
    pairs, invalid, skew_rejected = [], 0, 0
    last_t = None
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"t_s", "truth_t_s", "vio_n", "vio_e", "vio_d", "truth_n", "truth_e", "truth_d"}
        if not required.issubset(reader.fieldnames or ()):
            raise ValueError("Independent truth required; missing columns: " + ", ".join(sorted(required-set(reader.fieldnames or ()))))
        for index, row in enumerate(reader, 2):
            t = float(row["t_s"])
            if not math.isfinite(t) or (last_t is not None and t <= last_t):
                raise ValueError(f"row {index}: t_s must be finite and strictly increasing")
            last_t = t
            if row.get("valid", "1") not in ("0", "1"):
                raise ValueError(f"row {index}: valid must be 0 or 1")
            if row.get("valid", "1") == "0":
                invalid += 1
                continue
            truth_t = float(row["truth_t_s"])
            vio = tuple(float(row["vio_"+axis]) for axis in "ned")
            truth = tuple(float(row["truth_"+axis]) for axis in "ned")
            if not all(math.isfinite(v) for v in (truth_t, *vio, *truth)):
                raise ValueError(f"row {index}: nonfinite valid observation")
            if abs(t-truth_t) > max_pair_skew:
                skew_rejected += 1
                continue
            target = None
            if any(row.get("target_"+axis, "") for axis in "ned"):
                target = tuple(float(row["target_"+axis]) for axis in "ned")
                if not all(math.isfinite(v) for v in target):
                    raise ValueError(f"row {index}: nonfinite target")
            pairs.append(dict(t=t, vio=vio, truth=truth, target=target, epoch=row.get("reference_epoch", "0")))
    if not pairs:
        raise ValueError("No valid synchronized VIO/truth pairs")
    errors = [math.dist(p["vio"], p["truth"]) for p in pairs]
    gaps, changes, events, segments = [], 0, [], []
    segment = [pairs[0]]
    for a, b in zip(pairs, pairs[1:]):
        dt = b["t"]-a["t"]
        if dt > max_gap:
            gaps.append(dict(from_s=a["t"], to_s=b["t"], duration_s=dt))
        error_step = math.dist(tuple(y-x for x, y in zip(a["vio"], b["vio"])),
                               tuple(y-x for x, y in zip(a["truth"], b["truth"])))
        changed = a["epoch"] != b["epoch"]
        if changed:
            changes += 1
            segments.append(segment)
            segment = []
        if changed or (dt <= max_gap and error_step > jump_threshold):
            events.append(dict(t_s=b["t"], reference_changed=changed, error_step_m=error_step,
                               continuity_observed=dt <= max_gap))
        segment.append(b)
    segments.append(segment)
    drift = []
    for samples in segments:
        a, b = samples[0], samples[-1]
        error_change = math.dist(tuple(y-x for x, y in zip(a["vio"], b["vio"])),
                                 tuple(y-x for x, y in zip(a["truth"], b["truth"])))
        drift.append(dict(epoch=a["epoch"], start_s=a["t"], end_s=b["t"],
                          vio_displacement_m=math.dist(a["vio"], b["vio"]),
                          true_displacement_m=math.dist(a["truth"], b["truth"]),
                          start_to_end_error_change_m=error_change))
    end = pairs[-1]
    return dict(paired_samples=len(pairs), invalid_samples=invalid, unsynchronized_samples=skew_rejected,
                error_rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)),
                error_p95_m=percentile(errors, .95), error_max_m=max(errors),
                endpoint_localization_error_m=errors[-1],
                endpoint_true_target_error_m=math.dist(end["truth"], end["target"]) if end["target"] else None,
                endpoint_estimated_target_error_m=math.dist(end["vio"], end["target"]) if end["target"] else None,
                reference_changes=changes, discontinuity_events=events, segments=drift,
                unobserved_intervals=gaps,
                note="Independent pre-aligned truth; no bias/scale fitting. Sparse pairs cannot establish continuous accuracy or absence of jumps.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=pathlib.Path)
    ap.add_argument("--output", type=pathlib.Path)
    ap.add_argument("--max-pair-skew", type=float, default=.05)
    ap.add_argument("--max-gap", type=float, default=.25)
    ap.add_argument("--jump-threshold", type=float, default=.5)
    args = ap.parse_args()
    for value in (args.max_pair_skew, args.max_gap, args.jump_threshold):
        if not math.isfinite(value) or value <= 0:
            ap.error("timing/jump limits must be finite and positive")
    try:
        result = json.dumps(summarize(args.csv, args.max_pair_skew, args.max_gap, args.jump_threshold),
                            ensure_ascii=False, indent=2, allow_nan=False)
    except (ValueError, OSError) as exc:
        ap.error(str(exc))
    if args.output:
        args.output.write_text(result+"\n", encoding="utf-8")
    print(result)


if __name__ == "__main__":
    main()
