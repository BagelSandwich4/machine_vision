"""Compare two runs. Run:  python3 compare.py run_A.csv run_B.csv"""
import csv
import sys


def load(path):
    with open(path) as f:
        return {r["tag_id"]: r["stop"] for r in csv.DictReader(f)} # makes a dict of tag id and stop number frmo a log file


if len(sys.argv) != 3:
    print("Usage: python3 compare_runs run_A.csv run_B.csv")
    raise SystemExit(2)

old, new = load(sys.argv[1]), load(sys.argv[2])
for tag in old:
    if tag not in new:
        print("MISSING:", tag, "(was at stop", old[tag] + ")")
    elif new[tag] != old[tag]:
        print("MOVED:", tag, "stop", old[tag], "->", new[tag])
for tag in new:
    if tag not in old:
        print("NEW:", tag, "(at stop", new[tag] + ")")
print("done")