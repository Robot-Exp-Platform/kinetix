#!/usr/bin/env python3
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


def read_text(path):
    for encoding in ("utf-8-sig", "utf-16"):
        try:
            return Path(path).read_text(encoding=encoding)
        except UnicodeError:
            continue
    return Path(path).read_text()


def read_values(path):
    contents = read_text(path)
    csv_lines = [
        line
        for line in contents.splitlines()
        if line.startswith("implementation,")
        or line.startswith("kinetix,")
        or line.startswith("pinocchio,")
    ]
    rows = {}
    for row in csv.DictReader(csv_lines):
        key = (
            row["group"],
            row["model"],
            row["quantity"],
            row["dof"],
            row["row"],
            row["col"],
        )
        rows[key] = float(row["value"])
    return rows


def main():
    if len(sys.argv) not in (3, 4):
        print(
            "usage: compare_correctness.py kinetix_correctness.csv pinocchio_correctness.csv [tolerance]",
            file=sys.stderr,
        )
        return 2

    tolerance = float(sys.argv[3]) if len(sys.argv) == 4 else 1.0e-9
    kinetix = read_values(sys.argv[1])
    pinocchio = read_values(sys.argv[2])

    missing_pinocchio = sorted(kinetix.keys() - pinocchio.keys())
    missing_kinetix = sorted(pinocchio.keys() - kinetix.keys())
    if missing_pinocchio or missing_kinetix:
        print("missing rows detected", file=sys.stderr)
        for key in missing_pinocchio[:20]:
            print("missing from pinocchio: " + ",".join(key), file=sys.stderr)
        for key in missing_kinetix[:20]:
            print("missing from kinetix: " + ",".join(key), file=sys.stderr)
        return 1

    stats = defaultdict(lambda: [0.0, 0.0, None])
    global_max = 0.0
    for key in sorted(kinetix):
        diff = abs(kinetix[key] - pinocchio[key])
        denom = max(abs(kinetix[key]), abs(pinocchio[key]), 1.0)
        rel = diff / denom
        summary_key = key[:4]
        if stats[summary_key][2] is None or diff > stats[summary_key][0]:
            stats[summary_key] = [diff, rel, key]
        global_max = max(global_max, diff)

    print("group,model,quantity,dof,max_abs,max_rel,row,col,status")
    failed = False
    for summary_key in sorted(stats):
        max_abs, max_rel, full_key = stats[summary_key]
        status = "ok" if max_abs <= tolerance else "fail"
        failed = failed or status == "fail"
        print(
            f"{summary_key[0]},{summary_key[1]},{summary_key[2]},{summary_key[3]},"
            f"{max_abs:.12e},{max_rel:.12e},{full_key[4]},{full_key[5]},{status}"
        )

    if failed:
        print(f"\nmax_abs={global_max:.12e} exceeds tolerance={tolerance:.12e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
