#!/usr/bin/env python3
"""Verify the preserved artifacts recorded before the final repository cleanup."""
import argparse
import csv
import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--all', action='store_true', help='Also check original local-only data; requires the original working datasets.')
    args = parser.parse_args()
    checked = 0
    failures = []
    with (ROOT / 'data/MANIFEST.csv').open(newline='') as handle:
        for row in csv.DictReader(handle):
            if not args.all and row['tracked'] != 'True':
                continue
            path = ROOT / row['current_path']
            if not path.is_file():
                failures.append(f'Missing: {row["current_path"]}')
                continue
            digest = hashlib.sha256()
            with path.open('rb') as artifact:
                for block in iter(lambda: artifact.read(1024 * 1024), b''):
                    digest.update(block)
            if digest.hexdigest() != row['sha256']:
                failures.append(f'Changed: {row["current_path"]}')
            else:
                checked += 1
    for failure in failures:
        print(failure, file=sys.stderr)
    print(f'{checked} artifacts verified; {len(failures)} failures.')
    return bool(failures)


if __name__ == '__main__':
    sys.exit(main())
