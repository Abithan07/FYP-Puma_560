#!/usr/bin/env python3
"""
triggered_logger.py
Records /joint_states to CSV during trajectory execution.

Activated via ROS2 services:
  /logger/start  (std_srvs/Trigger) — begin recording
  /logger/stop   (std_srvs/Trigger) — stop and flush

Launched automatically by torque_publisher.py. Can also be run standalone.

Output CSV columns:
  time_elapsed, pos1, pos2, pos3, vel1, vel2, vel3, torque1, torque2, torque3

Usage (standalone):
  python3 triggered_logger.py --log-dir ~/puma560_logs
  python3 triggered_logger.py --dataset-path path_461_joint_states.csv \\
                               --log-dir ~/puma560_logs
"""

import argparse
import csv
import os
import signal
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class TriggeredLogger(Node):

    def __init__(self, dataset_path: Optional[str], log_dir: str):
        super().__init__('triggered_logger')

        # Derive a short tag from the dataset filename (e.g. "path_461")
        self.tag = 'traj'
        if dataset_path:
            base = os.path.splitext(os.path.basename(dataset_path))[0]
            parts = base.split('_')
            self.tag = '_'.join(parts[:2]) if len(parts) >= 2 else base

        os.makedirs(log_dir, exist_ok=True)
        self.log_dir = log_dir

        # Auto-increment log number
        n = 1
        while os.path.exists(os.path.join(log_dir, f'{self.tag}_log_{n}.csv')):
            n += 1
        self.csv_path = os.path.join(log_dir, f'{self.tag}_log_{n}.csv')

        self.get_logger().info(f'Log file: {self.csv_path}')

        # State
        self.logging  = False
        self.start_t  = None
        self.samples  = 0
        self.csv_file = None
        self.writer   = None

        # Latest joint data
        self.pos    = [0.0, 0.0, 0.0]
        self.vel    = [0.0, 0.0, 0.0]
        self.effort = [0.0, 0.0, 0.0]
        self.data_ok = False

        # Subscriber
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # 100 Hz log timer
        self.create_timer(0.01, self._log_cb)

        # Services
        self.create_service(Trigger, '/logger/start', self._start_cb)
        self.create_service(Trigger, '/logger/stop',  self._stop_cb)

        signal.signal(signal.SIGINT,  self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        self.get_logger().info('Waiting for /logger/start ...')

    # ── Joint state callback ──────────────────────────────────────────────
    def _js_cb(self, msg):
        try:
            i1 = msg.name.index('joint_1')
            i2 = msg.name.index('joint_2')
            i3 = msg.name.index('joint_3')
            self.pos = [msg.position[i1], msg.position[i2], msg.position[i3]]
            if len(msg.velocity) > max(i1, i2, i3):
                self.vel = [msg.velocity[i1], msg.velocity[i2], msg.velocity[i3]]
            if len(msg.effort) > max(i1, i2, i3):
                self.effort = [msg.effort[i1], msg.effort[i2], msg.effort[i3]]
            self.data_ok = True
        except (ValueError, IndexError):
            pass

    # ── 100 Hz log timer ──────────────────────────────────────────────────
    def _log_cb(self):
        if not self.logging or not self.data_ok:
            return
        elapsed = (self.get_clock().now() - self.start_t).nanoseconds * 1e-9
        self.writer.writerow([
            f'{elapsed:.4f}',
            f'{self.pos[0]:.6f}',    f'{self.pos[1]:.6f}',    f'{self.pos[2]:.6f}',
            f'{self.vel[0]:.6f}',    f'{self.vel[1]:.6f}',    f'{self.vel[2]:.6f}',
            f'{self.effort[0]:.6f}', f'{self.effort[1]:.6f}', f'{self.effort[2]:.6f}',
        ])
        self.samples += 1
        if self.samples % 500 == 0:
            self.csv_file.flush()

    # ── Service: start ─────────────────────────────────────────────────────
    def _start_cb(self, _, response):
        if self.logging:
            response.success = False
            response.message = 'Already logging.'
            return response

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer   = csv.writer(self.csv_file)
        self.writer.writerow([
            'time_elapsed',
            'pos1', 'pos2', 'pos3',
            'vel1', 'vel2', 'vel3',
            'torque1', 'torque2', 'torque3',
        ])
        self.start_t = self.get_clock().now()
        self.samples = 0
        self.logging = True
        self.get_logger().info('Logging STARTED.')
        response.success = True
        response.message = f'Logging to {self.csv_path}'
        return response

    # ── Service: stop ──────────────────────────────────────────────────────
    def _stop_cb(self, _, response):
        if not self.logging:
            response.success = False
            response.message = 'Not logging.'
            return response

        self.logging = False
        elapsed = (self.get_clock().now() - self.start_t).nanoseconds * 1e-9
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

        self.get_logger().info(
            f'Logging STOPPED — {self.samples} samples, {elapsed:.2f} s, '
            f'saved to {self.csv_path}')
        response.success = True
        response.message = f'{self.samples} samples in {elapsed:.2f} s'
        return response

    def _sig(self, *_):
        if self.logging and self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Triggered joint-state logger')
    parser.add_argument('--dataset-path', default=None,
                        help='Dataset CSV path (used only to derive the log filename)')
    parser.add_argument('--log-dir', default=os.path.expanduser('~/puma560_logs'),
                        help='Directory for output log files')
    args = parser.parse_args()

    rclpy.init()
    node = TriggeredLogger(dataset_path=args.dataset_path, log_dir=args.log_dir)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
