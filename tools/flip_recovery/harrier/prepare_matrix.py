"""Resolve archived Harrier test fixtures for the SITL batch runner."""
import argparse
import json
from pathlib import Path

root = Path(__file__).resolve().parents[3]
archive = root / "docs/verification/harrier_flip_20261122/results"
batches = {
    "flat": "closed_final_20261122",
    "slope": "closed_slope_20261122",
    "steep": "closed_steep_20261122",
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("batch", choices=batches)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    source = archive / batches[a.batch]
    fixtures = source / "fixtures"
    entries = json.loads((source / "matrix.json").read_text())
    for entry in entries:
        for field in ("model", "world"):
            ref = entry.get(field)
            if ref is None:
                continue
            if Path(ref).name != ref or not (fixtures / ref).is_file():
                p.error(f"invalid archived {field}: {ref}")
            entry[field] = str(fixtures / ref)
    with a.output.open("x", encoding="utf-8") as file:
        json.dump(entries, file, indent=2)
        file.write("\n")


if __name__ == "__main__":
    main()
