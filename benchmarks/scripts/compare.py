#!/usr/bin/env python3
import csv
import sys
from pathlib import Path


def read_csv(path):
    rows = {}
    contents = None
    for encoding in ("utf-8-sig", "utf-16"):
        try:
            contents = Path(path).read_text(encoding=encoding)
            break
        except UnicodeError:
            continue
    if contents is None:
        contents = Path(path).read_text()

    csv_lines = [
        line
        for line in contents.splitlines()
        if line.startswith("implementation,")
        or line.startswith("kinetix,")
        or line.startswith("pinocchio,")
    ]
    reader = csv.DictReader(csv_lines)
    for row in reader:
        key = (row["group"], row["model"], row["algorithm"], row["dof"])
        rows[key] = row
    return rows


def main():
    if len(sys.argv) != 3:
        print("usage: compare.py kinetix.csv pinocchio.csv", file=sys.stderr)
        return 2

    kinetix = read_csv(sys.argv[1])
    pinocchio = read_csv(sys.argv[2])

    print("group,model,algorithm,dof,kinetix_ns,pinocchio_ns,kinetix_speedup,error")
    for key in sorted(kinetix.keys() & pinocchio.keys()):
        k = kinetix[key]
        p = pinocchio[key]
        kinetix_ns = float(k["ns_per_iter"])
        pinocchio_ns = float(p["ns_per_iter"])
        speedup = pinocchio_ns / kinetix_ns if kinetix_ns > 0.0 else float("inf")
        checksum_error = abs(float(k["checksum"]) - float(p["checksum"]))
        print(
            f"{key[0]},{key[1]},{key[2]},{key[3]},"
            f"{kinetix_ns:.3f},{pinocchio_ns:.3f},{speedup:.3f},{checksum_error:.12e}"
        )

    missing_pinocchio = sorted(kinetix.keys() - pinocchio.keys())
    if missing_pinocchio:
        print("\n# rows missing from pinocchio:", file=sys.stderr)
        for key in missing_pinocchio:
            print("# " + ",".join(key), file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
