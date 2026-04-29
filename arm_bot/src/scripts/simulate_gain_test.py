#!/usr/bin/env python3
"""
Recompute feedback torques for different gains using logged errors and the
offline inverse dynamics model. Produces a CSV with tau_fb_old and tau_fb_new.
"""
import argparse
from pathlib import Path
import numpy as np
import pandas as pd

from inverse_dynamics_model import compute_inverse_dynamics

def run(log_path, out_path, kp_scale=1.0, kd_scale=1.0, ki_scale=1.0,
        kp_override=None, kd_override=None, ki_override=None):
    df = pd.read_csv(log_path)

    # Default controller gains used in node
    kp_base = np.array([30.0, 100.0, 50.0]) if kp_override is None else np.array(kp_override)
    kd_base = np.array([4.0, 10.0, 5.0]) if kd_override is None else np.array(kd_override)
    ki_base = np.array([0.2, 1.0, 0.8]) if ki_override is None else np.array(ki_override)

    kp_new = kp_base * kp_scale
    kd_new = kd_base * kd_scale
    ki_new = ki_base * ki_scale

    n = len(df)

    # Reconstruct feedback virtual accel and tau_fb for each timestep
    tau_model = np.zeros((n,3))
    tau_fb_old = np.zeros((n,3))
    tau_fb_new = np.zeros((n,3))

    # For integral term we approximate using cumulative sum of logged e_pos*dt
    t = df['t'].to_numpy()
    dt = t[1]-t[0] if len(t)>1 else 0.01
    e_pos = np.vstack([df[f'e_pos_{j}'].to_numpy() for j in [1,2,3]]).T
    e_vel = np.vstack([df[f'e_vel_{j}'].to_numpy() for j in [1,2,3]]).T
    integral = np.cumsum(e_pos, axis=0) * dt

    for i in range(n):
        q_des = np.array([df[f'q_des_{j}'].iloc[i] for j in [1,2,3]])
        qd_des = np.array([df[f'qd_des_{j}'].iloc[i] for j in [1,2,3]])
        qdd_des = np.array([df[f'qdd_des_{j}'].iloc[i] for j in [1,2,3]])

        # old feedback accel (using recorded node gains)
        vfb_old = kp_base * e_pos[i] + kd_base * e_vel[i] + ki_base * integral[i]
        vfb_new = kp_new * e_pos[i] + kd_new * e_vel[i] + ki_new * integral[i]

        # virtual accelerations
        v_old = qdd_des + vfb_old
        v_new = qdd_des + vfb_new

        tau_m, D, C, G = compute_inverse_dynamics(q_des, qd_des, qdd_des)
        tau_total_new, _, _, _ = compute_inverse_dynamics(q_des, qd_des, v_new)
        tau_total_old, _, _, _ = compute_inverse_dynamics(q_des, qd_des, v_old)

        tau_model[i,:] = tau_m
        tau_fb_old[i,:] = tau_total_old - tau_m
        tau_fb_new[i,:] = tau_total_new - tau_m

    out = pd.DataFrame({
        't': t,
    })
    for j in [1,2,3]:
        out[f'tau_fb_old_{j}'] = tau_fb_old[:,j-1]
        out[f'tau_fb_new_{j}'] = tau_fb_new[:,j-1]
        out[f'tau_model_{j}'] = tau_model[:,j-1]

    out.to_csv(out_path, index=False)
    print(f'Wrote {out_path} (n={n})')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('log', help='CTC log CSV (feedback on)')
    p.add_argument('--out', default='gain_test_compare.csv')
    p.add_argument('--kp-scale', type=float, default=1.0)
    p.add_argument('--kd-scale', type=float, default=1.0)
    p.add_argument('--ki-scale', type=float, default=1.0)
    p.add_argument('--kp3', type=float, default=None, help='override kp for joint3')
    p.add_argument('--kd3', type=float, default=None, help='override kd for joint3')
    args = p.parse_args()

    kp_override = None
    kd_override = None
    if args.kp3 is not None:
        kp_override = [30.0, 100.0, args.kp3]
    if args.kd3 is not None:
        kd_override = [4.0, 10.0, args.kd3]

    run(Path(args.log), Path(args.out), kp_scale=args.kp_scale, kd_scale=args.kd_scale,
        ki_scale=args.ki_scale, kp_override=kp_override, kd_override=kd_override)

if __name__ == '__main__':
    main()
