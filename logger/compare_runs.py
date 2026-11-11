"""
Compare two tag logs and report NEW, MISSING, and MOVED tags.

Both CSVs must be in the current folder. Run:
    python3 compare_runs.py run_A.csv run_B.csv
"""
import csv
import sys


def load(path):
    """Return a dict mapping tag ID to stop number from a log file."""
    with open(path) as f:
        return {r['tag_id']: r['stop'] for r in csv.DictReader(f)}


if len(sys.argv) != 3:
    print('Usage: python3 compare_runs.py run_A.csv run_B.csv')
    raise SystemExit(2)

old, new = load(sys.argv[1]), load(sys.argv[2])
for tag in old:
    if tag not in new:
        print('MISSING:', tag, '(was at stop', old[tag] + ')')
    elif new[tag] != old[tag]:
        print('MOVED:', tag, 'stop', old[tag], '->', new[tag])
for tag in new:
    if tag not in old:
        print('NEW:', tag, '(at stop', new[tag] + ')')
print('done')
