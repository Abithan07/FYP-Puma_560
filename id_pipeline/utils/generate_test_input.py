#!/usr/bin/env python3
"""
Generate a synthetic test input CSV for the ID pipeline.
Produces smooth sinusoidal joint trajectories at 1000 Hz.

Usage:
  python3 generate_test_input.py --out /path/to/test_input.csv [--duration 5]
"""

import argparse
import csv
import math
import os

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out',      default='test_input.csv')
    ap.add_argument('--duration', type=float, default=5.0, help='seconds')
    ap.add_argument('--dt',       type=float, default=0.001, help='timestep (s)')
    args = ap.parse_args()

    dt    = args.dt
    T     = args.duration
    steps = int(round(T / dt))

    # Joint amplitudes (rad) and frequencies (rad/s)
    amp  = [0.5,   0.4,   0.6]
    freq = [0.5,   0.8,   1.2]   # Hz
    off  = [0.0,   math.pi/4, 3*math.pi/4]  # rad, match URDF initial values

    rows = {k: [] for k in ['t','q1','q2','q3','dq1','dq2','dq3']}

    for k in range(steps):
        t = k * dt
        rows['t'].append(t)
        for j in range(3):
            w = 2 * math.pi * freq[j]
            q  =  off[j] + amp[j] * math.sin(w * t)
            dq =  amp[j] * w * math.cos(w * t)
            rows[f'q{j+1}'].append(q)
            rows[f'dq{j+1}'].append(dq)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        writer = csv.writer(f)
        for key, vals in rows.items():
            writer.writerow([key] + vals)

    print(f'Generated {steps} steps ({T:.1f} s at {1.0/dt:.0f} Hz) → {args.out}')

if __name__ == '__main__':
    main()
