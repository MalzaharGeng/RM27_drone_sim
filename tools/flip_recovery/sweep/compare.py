"""Compare confirmation cases only where all four configurations are comparable."""
import argparse
import csv
import json
from pathlib import Path


ROLES = ("off","default","candidate","auto")
DISPLAY_ROLES = (*ROLES,"low")
VALID = {"PASS","DISABLED_STAYED_TIPPED","PASSIVE_UPRIGHT","NO_TRUTH_RECOVERY",
         "NO_DRIVE","NO_MOTOR","TRUTH_ONLY"}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directory", type=Path)
    a = ap.parse_args()
    rows = json.loads((a.directory/"results.json").read_text())
    compare = {}
    for r in rows:
        if r["batch"] not in {"confirm1","confirm_auto1","confirm_low1"}:
            continue
        name, role = r["id"].rsplit("_",1)
        if role in DISPLAY_ROLES:
            compare.setdefault(name,{})[role] = r
    output = []
    for name, runs in sorted(compare.items()):
        row = {"scenario":name}
        base = {role:r for role,r in runs.items() if role in ROLES}
        for role in DISPLAY_ROLES:
            r = runs.get(role,{})
            row |= {f"{role}_{key}":r.get(key) for key in ("outcome","start_deg","final_deg","reason","dwell_s")}
        if not set(ROLES) <= base.keys() or any(r["outcome"] not in VALID for r in base.values()):
            row["paired"] = "EXCLUDED"
        else:
            angles = [r["start_deg"] for r in base.values()]
            if max(angles)-min(angles) > 1:
                row["paired"] = "MISMATCH"
            elif runs["off"]["outcome"] != "DISABLED_STAYED_TIPPED":
                row["paired"] = "PASSIVE"
            elif min(angles)>170:
                row["paired"] = "INVERTED"
            elif 65 < min(angles) and max(angles)<90:
                row["paired"] = "SIDE"
            else:
                row["paired"] = "OTHER"
        low = runs.get("low")
        row["low_matched"] = bool(low and low["outcome"] in VALID and row["paired"] == "SIDE"
                                  and abs(low["start_deg"]-base["off"]["start_deg"]) <= 1)
        output.append(row)
    with (a.directory/"paired_confirmation.csv").open("w",newline="") as f:
        w = csv.DictWriter(f,list(output[0]))
        w.writeheader()
        w.writerows(output)
    for cluster in ("SIDE","INVERTED","OTHER","MISMATCH","PASSIVE","EXCLUDED"):
        found = [r for r in output if r["paired"]==cluster]
        if found:
            print(cluster, len(found),
                  {role:sum(r[f"{role}_outcome"]=="PASS" for r in found) for role in ROLES})
    low_cases = [r for r in output if r["low_matched"]]
    if low_cases:
        print("SIDE low", sum(r["low_outcome"]=="PASS" for r in low_cases), len(low_cases))


if __name__ == "__main__":
    main()
