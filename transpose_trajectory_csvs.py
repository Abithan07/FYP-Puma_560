#!/usr/bin/env python3
"""Rotate trajectory CSV files into labeled signal rows.

Each input file is expected to contain 10 trajectory signals in this order:
t, dp1, dp2, dp3, dv1, dv2, dv3, da1, da2, da3

Files in `Test_1` are stored as time-step rows with 10 numeric columns. This
script rotates them so each signal becomes a row, with the signal name in the
first column.
"""

from __future__ import annotations

import csv
from pathlib import Path


IN_DIR = Path("/home/priyankan/Desktop/FYP-Puma_560/Test_1")
OUT_DIR = Path("/home/priyankan/Desktop/FYP-Puma_560/Test_1_1")

HEADERS = ["t", "dp1", "dp2", "dp3", "dv1", "dv2", "dv3", "da1", "da2", "da3"]


def normalize_csv_file(input_path: Path, output_path: Path) -> None:
    with input_path.open(newline="") as infile:
        rows = [row for row in csv.reader(infile) if row]

    if not rows:
        return

    row_count = len(rows)
    column_count = len(rows[0])

    for row in rows:
        if len(row) != column_count:
            raise ValueError(f"Inconsistent row length in {input_path.name}")

    if column_count != len(HEADERS):
        raise ValueError(
            f"Unsupported shape in {input_path.name}: {row_count} rows x {column_count} columns"
        )

    output_rows = [[label, *values] for label, values in zip(HEADERS, zip(*rows))]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as outfile:
        writer = csv.writer(outfile)
        writer.writerows(output_rows)

    print(f"Converted {input_path.name}: {row_count}x{column_count}")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    csv_files = sorted(IN_DIR.glob("*.csv"))
    if not csv_files:
        print(f"No CSV files found in {IN_DIR}")
        return

    for input_path in csv_files:
        output_path = OUT_DIR / input_path.name
        normalize_csv_file(input_path, output_path)


if __name__ == "__main__":
    main()