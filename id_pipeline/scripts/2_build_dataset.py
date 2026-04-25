#!/usr/bin/env python3
"""
Inverse Dynamics Pipeline – Step 2: Build Dataset
==================================================
Reads the raw intermediate CSV produced by 1_trajectory_player.py and
produces a clean inverse-dynamics dataset CSV in the same row-major
format as the input, with three new rows appended: tau1, tau2, tau3.

Why tau_cmd is the TRUE inverse dynamics torque
-----------------------------------------------
Newton-Euler equation of motion for the robot:

    τ_applied = M(q)·q̈ + C(q,q̇)·q̇ + G(q)   ← definition of τ_ID

The ForwardCommandController sends tau_cmd directly to the physics engine
as τ_applied.  Therefore, by Newton-Euler:

    tau_cmd  =  τ_ID(q_act, dq_act, ddq_act)   ← exact identity

The PD correction inside tau_cmd (Kp·e + Kd·ė) is NOT a controller
artefact — it IS the inertia+Coriolis term that produces the actual
joint acceleration.  Subtracting it would destroy the inverse dynamics,
leaving only the gravity component.  Do NOT subtract it.

The only noise in tau_cmd comes from the physics solver's numerical
integration, which is removed by the Butterworth low-pass filter below.

Processing chain
----------------
1. Load raw CSV → tau_cmd (true τ_ID) and actual state (q_act, dq_act).
2. Savitzky-Golay differentiation of q_act → ddq_act (smooth, no lag).
3. Zero-phase Butterworth low-pass filter on tau_cmd to suppress
   physics-solver high-frequency numerical noise (typically > 50 Hz)
   while preserving the inverse-dynamics signal (< 10–20 Hz).
4. Write output CSV:
       rows: t, q1, q2, q3, dq1, dq2, dq3, ddq1, ddq2, ddq3,
             tau1, tau2, tau3
   q/dq rows use the COMMANDED reference values (consistent with the
   original input format and with model deployment, since tracking is
   tight enough that q_ref ≈ q_act to < 0.1°).
   ddq is always derived from q_act for physical accuracy.
   tau is always tau_cmd = true τ_ID from the Gazebo physics engine.

No ROS2 required – pure Python/NumPy/SciPy post-processing.

Usage:
  python3 2_build_dataset.py \\
      --raw   /path/to/raw.csv \\
      --input /path/to/original_input.csv \\
      --out   /path/to/dataset.csv \\
      [--cutoff 30] [--sg-window 21] [--sg-order 4] \\
      [--subtract-pd] [--plot]
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter


# ─── CSV I/O ──────────────────────────────────────────────────────────────────

def load_col_csv(path: str) -> dict[str, np.ndarray]:
    """Load column-major CSV (header row names, data rows below)."""
    data: dict[str, np.ndarray] = {}
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        cols: dict[str, list] = {k: [] for k in reader.fieldnames}
        for row in reader:
            for k in reader.fieldnames:
                cols[k].append(float(row[k]))
    return {k: np.array(v) for k, v in cols.items()}


def load_row_csv(path: str) -> dict[str, np.ndarray]:
    """Load row-major CSV (first token is label, rest are values)."""
    data: dict[str, np.ndarray] = {}
    with open(path, 'r') as f:
        for row in csv.reader(f):
            if len(row) < 2:
                continue
            key = row[0].strip()
            try:
                data[key] = np.array([float(v) for v in row[1:]])
            except ValueError:
                pass
    # Normalise legacy names
    for old, new in [('dp1','q1'),('dp2','q2'),('dp3','q3'),
                     ('dv1','dq1'),('dv2','dq2'),('dv3','dq3')]:
        if old in data and new not in data:
            data[new] = data.pop(old)
    return data


def write_row_csv(path: str, rows: dict[str, np.ndarray]) -> None:
    """Write row-major CSV preserving full float precision."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        for key, vals in rows.items():
            writer.writerow([key] + [repr(v) for v in vals.tolist()])


# ─── Signal processing ────────────────────────────────────────────────────────

def savgol_ddq(q: np.ndarray, dt: float,
               window: int, order: int) -> np.ndarray:
    """
    Compute second derivative using Savitzky-Golay filter.
    Window must be odd and > order+2.  If the signal is shorter than
    window, the window is shrunk to fit (must still be > 3).
    """
    n = len(q)
    w = window
    if w > n:
        w = n if n % 2 == 1 else n - 1
        w = max(w, order + 2 + (1 if (order + 2) % 2 == 0 else 0))
    if w % 2 == 0:
        w += 1
    return savgol_filter(q, window_length=w, polyorder=order,
                         deriv=2, delta=dt)


def butterworth_lpf(signal: np.ndarray, dt: float, cutoff_hz: float,
                    order: int = 4) -> np.ndarray:
    """
    Zero-phase Butterworth low-pass filter (filtfilt → no phase shift).
    Cutoff is automatically clamped to 0.45 * Nyquist so it stays valid.
    """
    fs = 1.0 / dt
    nyq = 0.5 * fs
    fc  = min(cutoff_hz, 0.45 * nyq)
    if fc <= 0:
        return signal.copy()
    b, a = butter(order, fc / nyq, btype='low')
    # filtfilt needs at least 3*max(len(a),len(b)) samples
    padlen = 3 * max(len(a), len(b))
    if len(signal) <= padlen:
        return signal.copy()
    return filtfilt(b, a, signal)


# ─── Quality diagnostics ─────────────────────────────────────────────────────

def print_quality_report(raw: dict, tau_out: list[np.ndarray],
                         dt: float) -> None:
    n = len(raw['t'])
    print('\n── Tracking quality ──────────────────────────────────')
    for i in range(1, 4):
        e = raw.get(f'e{i}', np.zeros(n))
        rmse  = np.sqrt(np.mean(e**2))
        maxe  = np.max(np.abs(e))
        print(f'  Joint {i}:  RMSE={math.degrees(rmse):.5f}°  '
              f'max={math.degrees(maxe):.5f}°')

    print('\n── Torque statistics (filtered) ──────────────────────')
    for i, tau in enumerate(tau_out):
        print(f'  tau{i+1}:  mean={np.mean(tau):.3f}  '
              f'std={np.std(tau):.3f}  '
              f'range=[{tau.min():.2f}, {tau.max():.2f}] Nm')

    print(f'\n── Dataset ────────────────────────────────────────────')
    print(f'  Samples: {n}    Duration: {n*dt:.3f} s    dt: {dt*1000:.3f} ms')
    print('────────────────────────────────────────────────────────\n')


def optional_plot(raw: dict, tau_filt: list[np.ndarray],
                  ddq: list[np.ndarray]) -> None:
    """Plot tracking errors, raw vs filtered torques, and ddq."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed – skipping plot.')
        return

    t = raw['t']
    fig, axes = plt.subplots(3, 3, figsize=(15, 9))
    fig.suptitle('ID Pipeline Quality', fontsize=12)

    joint_labels = ['Joint 1', 'Joint 2', 'Joint 3']

    for i in range(3):
        ax_err, ax_tau, ax_ddq = axes[i]

        # Tracking error
        e = raw.get(f'e{i+1}', np.zeros(len(t)))
        ax_err.plot(t, np.degrees(e), lw=0.8)
        ax_err.set_ylabel('Error (°)')
        ax_err.set_title(f'{joint_labels[i]} pos error')
        ax_err.axhline(0, color='k', lw=0.5)
        ax_err.grid(True, alpha=0.3)

        # Torque: raw cmd vs filtered
        tau_raw = raw.get(f'tau{i+1}_cmd', np.zeros(len(t)))
        ax_tau.plot(t, tau_raw, alpha=0.4, lw=0.6, label='raw')
        ax_tau.plot(t, tau_filt[i], lw=1.2, label='filtered', color='tab:red')
        ax_tau.set_ylabel('τ (Nm)')
        ax_tau.set_title(f'{joint_labels[i]} torque')
        ax_tau.legend(fontsize=8)
        ax_tau.grid(True, alpha=0.3)

        # ddq
        ax_ddq.plot(t, ddq[i], lw=0.8, color='tab:green')
        ax_ddq.set_ylabel('ddq (rad/s²)')
        ax_ddq.set_title(f'{joint_labels[i]} acceleration')
        ax_ddq.grid(True, alpha=0.3)

    for ax in axes[-1]:
        ax.set_xlabel('time (s)')

    plt.tight_layout()
    plt.show()


# ─── Default paths ────────────────────────────────────────────────────────────

_TRAJ_DIR = os.path.expanduser(
    '~/Desktop/FYP-Puma_560/Dataset/Trajectories'
)


def _resolve_paths(args) -> tuple[str, str, str]:
    """Return (raw_csv, input_csv, out_csv) resolved from --id or explicit flags."""
    if args.id is not None:
        fid       = f'{args.id:03d}'
        raw_csv   = f'/tmp/id_raw_{fid}.csv'
        input_csv = os.path.join(_TRAJ_DIR, f'path_{fid}_trajectory.csv')
        out_csv   = os.path.join(_TRAJ_DIR, f'path_{fid}_dataset.csv')
    else:
        missing = [n for n, v in [('--raw', args.raw),
                                   ('--input', args.input),
                                   ('--out', args.out)] if v is None]
        if missing:
            raise SystemExit(f'Provide either --id N or all of: {missing}')
        raw_csv, input_csv, out_csv = args.raw, args.input, args.out
    return raw_csv, input_csv, out_csv


# ─── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description='ID Pipeline Step 2 – post-process raw data into dataset',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  python3 2_build_dataset.py --id 1\n'
            '  python3 2_build_dataset.py --id 3 --cutoff 20 --plot\n'
            '  python3 2_build_dataset.py --raw /tmp/r.csv --input traj.csv --out ds.csv\n'
        ),
    )
    ap.add_argument('--id', type=int, default=None,
                    metavar='N',
                    help='Trajectory file ID (e.g. 1 → path_001_trajectory.csv). '
                         'Resolves --raw, --input and --out automatically.')
    ap.add_argument('--raw',   default=None,
                    help='Raw intermediate CSV from step 1 (overrides --id)')
    ap.add_argument('--input', default=None,
                    help='Original trajectory CSV (overrides --id)')
    ap.add_argument('--out',   default=None,
                    help='Output dataset CSV path (overrides --id)')
    ap.add_argument('--cutoff',    type=float, default=30.0,
                    help='Butterworth low-pass cutoff Hz (default 30; use 20 for 100 Hz data)')
    ap.add_argument('--sg-window', type=int,   default=21,
                    help='Savitzky-Golay window for ddq, must be odd (default 21)')
    ap.add_argument('--sg-order',  type=int,   default=4,
                    help='Savitzky-Golay polynomial order (default 4)')
    ap.add_argument('--plot', action='store_true',
                    help='Show diagnostic plots after processing')
    args = ap.parse_args()

    raw_path, input_path, out_path = _resolve_paths(args)
    print(f'Raw    : {raw_path}')
    print(f'Input  : {input_path}')
    print(f'Output : {out_path}')

    # ── Load raw data ─────────────────────────────────────────────────────────
    print(f'\nLoading raw data …')
    raw = load_col_csv(raw_path)
    n   = len(raw['t'])
    dt  = float(np.mean(np.diff(raw['t'])))
    print(f'  {n} samples  dt={dt*1000:.3f} ms  ({1.0/dt:.1f} Hz)')

    # ── Load original trajectory CSV (preserves all original rows) ────────────
    print(f'Loading original trajectory …')
    orig_raw: dict[str, list] = {}
    orig_row_order: list[str] = []
    with open(input_path, 'r') as f:
        for row in csv.reader(f):
            if len(row) < 2:
                continue
            key = row[0].strip()
            orig_row_order.append(key)
            try:
                orig_raw[key] = [float(v) for v in row[1:]]
            except ValueError:
                orig_raw[key] = row[1:]  # keep as-is if non-numeric

    # Normalised view (q1/q2/q3 etc.) for internal use only
    orig = load_row_csv(input_path)
    t_orig = orig['t']
    print(f'  {len(t_orig)} columns  rows: {orig_row_order}')

    # ── Torque source: tau_cmd = true τ_ID ───────────────────────────────────
    # tau_cmd is what the physics engine applied.  By Newton-Euler:
    #   tau_cmd = M(q_act)·ddq_act + C(q_act,dq_act)·dq_act + G(q_act)
    # This IS the inverse dynamics torque — not a controller approximation.
    tau_cmd = [raw[f'tau{i+1}_cmd'] for i in range(3)]
    print('  tau_cmd = true inverse dynamics torque (Newton-Euler identity)')

    # ── Low-pass filter torques ───────────────────────────────────────────────
    t_raw = raw['t']
    print(f'Filtering torques: Butterworth LP  cutoff={args.cutoff} Hz  '
          f'order=4  zero-phase …')
    tau_filt = [butterworth_lpf(tau_cmd[i], dt, args.cutoff) for i in range(3)]

    # ── Savitzky-Golay differentiation for ddq ────────────────────────────────
    # Always use q_act: physically consistent with tau_cmd via Newton-Euler.
    print(f'Computing ddq via Savitzky-Golay on q_act  '
          f'window={args.sg_window}  order={args.sg_order} …')
    q_for_ddq = [raw[f'q{i+1}_act'] for i in range(3)]
    ddq = [savgol_ddq(q_for_ddq[i], dt, args.sg_window, args.sg_order)
           for i in range(3)]

    # ── Quality report ────────────────────────────────────────────────────────
    print_quality_report(raw, tau_filt, dt)

    # ── Resample to original timeline ─────────────────────────────────────────
    # If controller ran slower than CSV (e.g. 100 Hz vs 1000 Hz), raw has fewer
    # rows.  Linear interpolation restores the original timeline.
    def resample(arr: np.ndarray) -> np.ndarray:
        if len(arr) == len(t_orig) and np.allclose(t_raw, t_orig, atol=1e-9):
            return arr
        return np.interp(t_orig, t_raw, arr)

    tau_out = [resample(tau_filt[i]) for i in range(3)]

    # ── Write output: ALL original rows preserved + tau1/tau2/tau3 appended ───
    # The original rows (t, dp1…dp3, dv1…dv3, da1…da3) are written unchanged.
    # tau1/tau2/tau3 are the Gazebo physics inverse-dynamics torques.
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        # Preserve every original row exactly
        for key in orig_row_order:
            writer.writerow([key] + [repr(v) if isinstance(v, float)
                                     else v for v in orig_raw[key]])
        # Append the three new tau rows
        for i in range(3):
            writer.writerow([f'tau{i+1}'] + [repr(v) for v in tau_out[i].tolist()])

    n_cols = len(t_orig)
    print(f'\nDataset written → {out_path}')
    print(f'  Original rows preserved : {orig_row_order}')
    print(f'  New rows appended       : tau1, tau2, tau3')
    print(f'  Columns per row         : {n_cols}')

    # ── Optional plot ─────────────────────────────────────────────────────────
    if args.plot:
        optional_plot(raw, tau_filt, ddq)


if __name__ == '__main__':
    main()
