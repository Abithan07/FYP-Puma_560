#!/usr/bin/env python3
"""Verify published datasets or the artifacts recorded before repository cleanup."""
import argparse
import csv
import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def verify_manifest(root, manifest, original_only=False):
    """Return the number verified and failures without modifying any artifacts."""
    checked = 0
    failures = []
    with manifest.open(newline='') as handle:
        for row in csv.DictReader(handle):
            if original_only and row['tracked'] != 'True':
                continue
            path = root / row['current_path']
            if not path.is_file():
                failures.append(f'Missing: {row["current_path"]}')
                continue
            if path.stat().st_size != int(row['bytes']):
                failures.append(f'Changed: {row["current_path"]}')
                continue
            digest = hashlib.sha256()
            with path.open('rb') as artifact:
                for block in iter(lambda: artifact.read(1024 * 1024), b''):
                    digest.update(block)
            if digest.hexdigest() != row['sha256']:
                failures.append(f'Changed: {row["current_path"]}')
            else:
                checked += 1
    return checked, failures


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--all', action='store_true', help='Check every original artifact, including local historical results.')
    selection.add_argument('--datasets', action='store_true', help='Check all published input datasets and model assets; works in a fresh clone.')
    args = parser.parse_args(argv)
    manifest_name = 'DATASET_MANIFEST.csv' if args.datasets else 'MANIFEST.csv'
    try:
        checked, failures = verify_manifest(
            ROOT, ROOT / 'data' / manifest_name,
            original_only=not (args.all or args.datasets))
    except (OSError, ValueError, KeyError) as error:
        print(f'Cannot verify {manifest_name}: {error}', file=sys.stderr)
        return 1
    for failure in failures:
        print(failure, file=sys.stderr)
    print(f'{checked} artifacts verified; {len(failures)} failures.')
    return bool(failures)


if __name__ == '__main__':
    sys.exit(main())
