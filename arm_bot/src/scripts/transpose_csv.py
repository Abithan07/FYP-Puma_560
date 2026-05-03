#!/usr/bin/env python3
"""
Transpose CSV files.

This script will transpose a single CSV file or all CSV files in a directory.
By default it writes files named <basename>_transposed.csv next to the
input file, or to a user-provided output directory.

Usage examples:
  # Transpose a single file and write to same directory
  python3 src/scripts/transpose_split_csv.py path/to/file.csv

  # Transpose all CSVs in a directory and write to outdir
  python3 src/scripts/transpose_split_csv.py path/to/dir -o outdir

Options:
  --suffix string   Suffix to append before .csv on output files (default: _transposed)
  -d, --delimiter   Input CSV delimiter (default: ,)
"""
import argparse
import csv
import os
from itertools import zip_longest
from typing import List


def transpose_rows(rows: List[List[str]]) -> List[List[str]]:
    if not rows:
        return []
    return [list(col) for col in zip_longest(*rows, fillvalue="")]


def transpose_file(input_path: str, outdir: str = None, delimiter: str = ",", suffix: str = "_transposed") -> str:
    if outdir is None:
        outdir = os.path.dirname(input_path) or "."
    os.makedirs(outdir, exist_ok=True)

    with open(input_path, newline="") as f:
        reader = csv.reader(f, delimiter=delimiter)
        rows = [list(r) for r in reader]

    if not rows:
        print(f"Skipping empty file: {input_path}")
        return ""

    transposed = transpose_rows(rows)
    base = os.path.splitext(os.path.basename(input_path))[0]
    out_name = f"{base}{suffix}.csv"
    out_path = os.path.join(outdir, out_name)

    with open(out_path, "w", newline="") as of:
        writer = csv.writer(of)
        for row in transposed[0:]:  # use 1: to skip header if needed, use 0: to include all rows
            writer.writerow(row)

    print(f"Wrote transposed: {out_path}")
    return out_path


def process_path(path: str, outdir: str = None, delimiter: str = ",", suffix: str = "_transposed"):
    if os.path.isdir(path):
        files = sorted([os.path.join(path, f) for f in os.listdir(path) if f.lower().endswith('.csv')])
        if not files:
            print(f"No CSV files found in directory: {path}")
            return
        for f in files:
            transpose_file(f, outdir=outdir, delimiter=delimiter, suffix=suffix)
    elif os.path.isfile(path):
        transpose_file(path, outdir=outdir, delimiter=delimiter, suffix=suffix)
    else:
        print(f"Path not found: {path}")


def main():
    p = argparse.ArgumentParser(description="Transpose CSV file(s) in-place or from a directory.")
    p.add_argument("input", help="Input CSV file or directory containing CSV files")
    p.add_argument("-o", "--outdir", help="Output directory (default: same directory as each input file)")
    p.add_argument("--suffix", default="_transposed", help="Suffix to append to output filenames before .csv")
    p.add_argument("-d", "--delimiter", default=",", help="CSV delimiter (default: ,)")
    args = p.parse_args()

    process_path(args.input, outdir=args.outdir, delimiter=args.delimiter, suffix=args.suffix)


if __name__ == "__main__":
    main()
