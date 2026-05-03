#!/usr/bin/env python3

import numpy as np
import argparse
import pandas as pd
import matplotlib
matplotlib.use('TkAgg')   # or Qt5Agg
import matplotlib.pyplot as plt
import sys


def read_csv(file_path):
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()
    return df


def plot_results(log_csv_path):
    log_data = read_csv(log_csv_path)

    # Extract signals (coerce to float where possible)
    time = log_data['t'].astype(float).to_numpy()

    tau_total_1 = log_data['tau_total_1'].astype(float).to_numpy()
    tau_total_2 = log_data['tau_total_2'].astype(float).to_numpy()
    tau_total_3 = log_data['tau_total_3'].astype(float).to_numpy()

    tau_delan_1 = log_data['tau_delan_1'].astype(float).to_numpy()
    tau_delan_2 = log_data['tau_delan_2'].astype(float).to_numpy()
    tau_delan_3 = log_data['tau_delan_3'].astype(float).to_numpy()

    tau_dnn_1 = log_data['tau_dnn_1'].astype(float).to_numpy()
    tau_dnn_2 = log_data['tau_dnn_2'].astype(float).to_numpy()
    tau_dnn_3 = log_data['tau_dnn_3'].astype(float).to_numpy()

    tau_fb_1 = log_data['tau_fb_1'].astype(float).to_numpy()
    tau_fb_2 = log_data['tau_fb_2'].astype(float).to_numpy()
    tau_fb_3 = log_data['tau_fb_3'].astype(float).to_numpy()

    # GRU = dnn - delan
    tau_gru_1 = tau_dnn_1 - tau_delan_1
    tau_gru_2 = tau_dnn_2 - tau_delan_2
    tau_gru_3 = tau_dnn_3 - tau_delan_3

    qd_act_1 = log_data['qd_act_1'].astype(float).to_numpy()
    qd_act_2 = log_data['qd_act_2'].astype(float).to_numpy()
    qd_act_3 = log_data['qd_act_3'].astype(float).to_numpy()


    # Power = tau * qd
    total_power_1 = tau_total_1 * qd_act_1
    total_power_2 = tau_total_2 * qd_act_2
    total_power_3 = tau_total_3 * qd_act_3

    # Power from feedback
    feedback_power_1 = tau_fb_1 * qd_act_1
    feedback_power_2 = tau_fb_2 * qd_act_2
    feedback_power_3 = tau_fb_3 * qd_act_3

    # Energy = integral of absolute power over time
    total_energy_1 = np.trapz(np.abs(total_power_1), time)
    total_energy_2 = np.trapz(np.abs(total_power_2), time)
    total_energy_3 = np.trapz(np.abs(total_power_3), time)

    # Energy from feedback
    energy_fb_1 = np.trapz(np.abs(feedback_power_1), time)
    energy_fb_2 = np.trapz(np.abs(feedback_power_2), time)
    energy_fb_3 = np.trapz(np.abs(feedback_power_3), time)

    total_energy = total_energy_1 + total_energy_2 + total_energy_3
    total_feedback_energy = energy_fb_1 + energy_fb_2 + energy_fb_3

    if total_energy > 0:
        feedback_percentage = (total_feedback_energy / total_energy) * 100.0
    else:
        feedback_percentage = 0.0

    # Plotting
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(time, tau_total_1, label='Total Torque', color='blue')
    axes[0].plot(time, tau_delan_1, label='Delan Torque', color='green')
    axes[0].plot(time, tau_fb_1, label='PID Feedback Torque', color='red')
    axes[0].fill_between(time, 0, tau_gru_1, color='brown', alpha=0.15)
    axes[0].plot(time, tau_gru_1, label='GRU Torque', color='brown')
    axes[0].set_title('Torques for Joint 1')
    axes[0].set_ylabel('Torque (Nm)')
    axes[0].grid()
    # Set light gray background for the plot area
    axes[0].set_facecolor("#d8d7d7")

    axes[1].plot(time, tau_total_2, label='Total Torque', color='blue')
    axes[1].plot(time, tau_delan_2, label='Delan Torque', color='green')
    axes[1].plot(time, tau_fb_2, label='PID Feedback Torque', color='red')
    axes[1].fill_between(time, 0, tau_gru_2, color='brown', alpha=0.15)
    axes[1].plot(time, tau_gru_2, label='GRU Torque', color='brown')
    axes[1].set_title('Torques for Joint 2')
    axes[1].set_ylabel('Torque (Nm)')
    axes[1].grid()
    axes[1].set_facecolor("#d8d7d7")

    axes[2].plot(time, tau_total_3, label='Total Torque', color='blue')
    axes[2].plot(time, tau_delan_3, label='Delan Torque', color='green')
    axes[2].plot(time, tau_fb_3, label='PID Feedback Torque', color='red')
    axes[2].fill_between(time, 0, tau_gru_3, color='brown', alpha=0.15)
    axes[2].plot(time, tau_gru_3, label='GRU Torque', color='brown')
    axes[2].set_title('Torques for Joint 3')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Torque (Nm)')
    axes[2].grid()
    axes[2].set_facecolor("#d8d7d7")

    # Reserve a larger footer area for legend and multi-line summary
    plt.subplots_adjust(bottom=0.32)

    summary_text = (
        f"Total energy: {total_energy:.6g} J\n"
        f"Feedback energy: {total_feedback_energy:.6g} J\n"
        f"Feedback contribution: {feedback_percentage:.2f}%"
    )

    fig.text(
        0.5,
        0.035,
        summary_text,
        ha='center',
        va='bottom',
        fontsize=12,
        linespacing=1.4,
        fontweight='bold',
        color='red',
    )


    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc='lower center',
        bbox_to_anchor=(0.5, 0.14),
        ncol=4,
        fontsize=12,
    )

    plt.tight_layout(rect=(0, 0.24, 1, 1))
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot torque results from CSV log file')
    parser.add_argument('--log-csv-path', type=str, required=True, help='Path to the CSV log file')
    args = parser.parse_args()
    if not args.log_csv_path:
        print('Error: --log-csv-path is required')
        sys.exit(1)
    plot_results(args.log_csv_path)