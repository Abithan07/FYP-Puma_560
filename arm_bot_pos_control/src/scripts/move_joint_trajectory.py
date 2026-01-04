#!/usr/bin/env python3
"""
move_joint_trajectory.py

ROS2 node to move the robot from a start joint-angle vector to an end joint-angle 
vector, hold at the end position for 2 seconds, then return to the start position.
Uses a PID-based effort controller (publishes torques) at 100 Hz.

Phases:
1. Go to start position (if provided) and stabilize
2. Move from start to end over specified duration
3. Hold at end for 2 seconds (with PID control, no stabilization wait)
4. Return from end to start over same duration
5. Stabilize at start position

Usage examples:
  ros2 run <pkg> move_joint_trajectory.py -- [start1,start2,start3] [end1,end2,end3] [duration]
  python3 move_joint_trajectory.py "0,0,0" "1.0,-0.5,2.0" 5.0

This node mirrors the style of the existing `csv_torque_publisher.py` and
`stabilize_zero_position.py` scripts in the repo.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import time
import math
from typing import List
from trajectory_logger import TrajectoryLogger


def _parse_vec(arg: str) -> List[float]:
    parts = [p.strip() for p in arg.replace('[', '').replace(']', '').split(',')]
    return [float(p) for p in parts if p != '']


class MoveJointTrajectory(Node):
    def __init__(self, start: List[float] = None, end: List[float] = None, duration: float = 5.0):
        super().__init__('move_joint_trajectory')

        # Publishers for each joint effort controller
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)

        # Subscriber to joint_states
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Trajectory inputs
        self.requested_start = start  # may be None (use current pos)
        self.end_position = end if end is not None else [0.0, 0.0, 0.0]
        self.duration = max(0.01, float(duration))

        # Current robot state
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False

        # Control params (same-style defaults as other scripts)
        self.kp = [30.0, 100.0, 50.0]
        self.ki = [0.5, 1.5, 0.9]
        self.kd = [8.0, 25.0, 10.0]
        self.integral_error = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        self.max_torques = [80.0, 150.0, 40.0]
        self.max_integral = [10.0, 25.0, 5.0]

        # Execution state
        # Phases: WAIT_START -> GOTO_START -> STABILIZING -> MOVING -> HOLDING_AT_END -> RETURNING -> FINAL_STABILIZING_AT_START -> DONE
        self.phase = 'WAIT_START'
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Trajectory bookkeeping
        self.start_position = None
        self.trajectory_start_time = None
        self.elapsed = 0.0
        self.hold_duration = 2.0  # seconds to hold at end position
        self.hold_start_time = None
        # Goto-start parameters (when a start vector is provided)
        self.goto_start_duration = 2.0  # seconds to interpolate from current pos to requested_start
        self.goto_start_threshold = math.radians(2.0)  # if within 2 degrees, skip goto-start

        # Stability criteria
        # Per-joint allowed position error (radians). By default each joint may
        # be within 5 degrees of the target to be considered stabilized.
        self.position_threshold = math.radians(5)  # 5 degrees (scalar) default
        # If you want per-joint different thresholds, set `self.position_thresholds`
        # to a list of three values (radians). Otherwise the scalar above is used.
        self.position_thresholds = None
        self.min_stabilization_time = 0.5
        self.stabilization_start_time = None
        self.is_stabilized = False

        # Initialize trajectory logger
        self.logger_csv = TrajectoryLogger(base_name="move_trajectory", output_dir="./logs")
        self.csv_file = self.logger_csv.start_logging()
        self.last_torques = [0.0, 0.0, 0.0]  # Store last commanded torques for logging

        self.get_logger().info(f'MoveJointTrajectory initialized: end={self.end_position}, duration={self.duration}s')
        self.get_logger().info(f'Logging to: {self.csv_file} (Test #{self.logger_csv.get_test_number()})')

    def joint_state_callback(self, msg: JointState):
        try:
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')
            self.current_joint_pos = [msg.position[idx1], msg.position[idx2], msg.position[idx3]]
            if len(msg.velocity) > 0:
                self.current_joint_vel = [msg.velocity[idx1], msg.velocity[idx2], msg.velocity[idx3]]
            self.joint_states_received = True
        except (ValueError, IndexError):
            pass

    def _start_stabilization(self):
        # Decide start position. If a requested start is provided and robot is
        # not already near it, first interpolate (GOTO_START) to the start, then stabilize.
        if self.requested_start is not None:
            # distance to requested start
            dist = max(abs(self.requested_start[i] - self.current_joint_pos[i]) for i in range(3))
            if dist > self.goto_start_threshold:
                # begin goto-start interpolation
                self.goto_start_initial = self.current_joint_pos.copy()
                self.goto_start_time = time.time()
                self.phase = 'GOTO_START'
                self.get_logger().info(f'Going to start position (interp {self.goto_start_duration}s): {self.requested_start}')
                return
            else:
                self.start_position = self.requested_start
        else:
            # use current position as start
            self.start_position = self.current_joint_pos.copy()

        self.get_logger().info(f'Starting stabilization at: [{self.start_position[0]:.4f}, {self.start_position[1]:.4f}, {self.start_position[2]:.4f}]')
        self.phase = 'STABILIZING'
        self.is_stabilized = False
        self.stabilization_start_time = None
        self.integral_error = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]

    def _compute_pid(self, errors):
        # update integral with anti-windup
        for i in range(3):
            self.integral_error[i] += errors[i] * self.dt
            self.integral_error[i] = max(-self.max_integral[i], min(self.max_integral[i], self.integral_error[i]))

        derivative = [(errors[i] - self.previous_error[i]) / self.dt for i in range(3)]

        torques = [
            self.kp[i] * errors[i] + self.ki[i] * self.integral_error[i] + self.kd[i] * derivative[i]
            for i in range(3)
        ]

        # saturate
        torques = [max(-self.max_torques[i], min(self.max_torques[i], torques[i])) for i in range(3)]
        self.previous_error = errors.copy()
        return torques

    def control_loop(self):
        if not self.joint_states_received:
            return

        if self.phase == 'WAIT_START':
            # Once joint states available, initiate stabilization
            self._start_stabilization()
            return

        if self.phase == 'GOTO_START':
            # Interpolate from current position to requested_start over goto_start_duration
            elapsed_gs = time.time() - self.goto_start_time
            s_gs = min(1.0, elapsed_gs / self.goto_start_duration)
            desired_gs = [self.goto_start_initial[i] + s_gs * (self.requested_start[i] - self.goto_start_initial[i]) for i in range(3)]
            errors = [desired_gs[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)
            if s_gs >= 1.0:
                # reached start; move to stabilization phase
                self.start_position = self.requested_start.copy()
                self.get_logger().info('Reached start position via GOTO_START. Now stabilizing.')
                self.phase = 'STABILIZING'
                self.stabilization_start_time = None
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]
            return

        if self.phase == 'STABILIZING':
            # Try to hold start_position using PID
            errors = [self.start_position[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            # Check per-joint threshold: require every joint error to be within
            # the allowed threshold. This prevents one small joint meeting the
            # criterion while others are still large.
            if self.position_thresholds is None:
                per_joint_ok = all(abs(e) < self.position_threshold for e in errors)
            else:
                per_joint_ok = all(abs(errors[i]) < self.position_thresholds[i] for i in range(3))

            if per_joint_ok:
                if self.stabilization_start_time is None:
                    self.stabilization_start_time = time.time()
                elif time.time() - self.stabilization_start_time >= self.min_stabilization_time:
                    self.get_logger().info('✓ START POSITION STABILIZED. Beginning trajectory.')
                    # begin trajectory
                    self.phase = 'MOVING'
                    self.trajectory_start_time = time.time()
                    self.elapsed = 0.0
                    # reset integrator for tracking
                    self.integral_error = [0.0, 0.0, 0.0]
                    self.previous_error = [0.0, 0.0, 0.0]
            else:
                self.stabilization_start_time = None

            return

        if self.phase == 'MOVING':
            self.elapsed = time.time() - self.trajectory_start_time
            s = min(1.0, self.elapsed / self.duration)
            desired = [self.start_position[i] + s * (self.end_position[i] - self.start_position[i]) for i in range(3)]
            # optional desired velocity (constant)
            desired_vel = [ (self.end_position[i] - self.start_position[i]) / self.duration for i in range(3) ]

            errors = [desired[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)

            # publish
            self._publish_torques(torques)

            # log periodically
            if int(self.elapsed / 0.5) != int((self.elapsed - self.dt) / 0.5):
                self.get_logger().info(f'MOVING... t={self.elapsed:.2f}/{self.duration:.2f} s | desired=[{desired[0]:.3f},{desired[1]:.3f},{desired[2]:.3f}] | err={max(abs(e) for e in errors):.4f} rad')

            if s >= 1.0:
                self.get_logger().info(f'✓ Trajectory complete. Holding at end position for {self.hold_duration}s...')
                self.phase = 'HOLDING_AT_END'
                self.final_position = self.end_position.copy()
                self.hold_start_time = time.time()
                # reset integrator
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]

            return

        if self.phase == 'HOLDING_AT_END':
            # Hold at end position for specified duration
            errors = [self.final_position[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            elapsed_hold = time.time() - self.hold_start_time
            if elapsed_hold >= self.hold_duration:
                self.get_logger().info('✓ Hold complete. Returning to start position...')
                self.phase = 'RETURNING'
                self.trajectory_start_time = time.time()
                self.elapsed = 0.0
                # reset integrator for return trajectory
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]

            return

        if self.phase == 'RETURNING':
            # Interpolate back from end to start
            self.elapsed = time.time() - self.trajectory_start_time
            s = min(1.0, self.elapsed / self.duration)
            desired = [self.end_position[i] + s * (self.start_position[i] - self.end_position[i]) for i in range(3)]

            errors = [desired[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            # log periodically
            if int(self.elapsed / 0.5) != int((self.elapsed - self.dt) / 0.5):
                self.get_logger().info(f'RETURNING... t={self.elapsed:.2f}/{self.duration:.2f} s | desired=[{desired[0]:.3f},{desired[1]:.3f},{desired[2]:.3f}] | err={max(abs(e) for e in errors):.4f} rad')

            if s >= 1.0:
                self.get_logger().info('✓ Return trajectory complete. Stabilizing at start position...')
                self.phase = 'FINAL_STABILIZING_AT_START'
                self.is_stabilized = False
                self.stabilization_start_time = None
                # reset integrator
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]

            return

        if self.phase == 'FINAL_STABILIZING_AT_START':
            errors = [self.start_position[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            # Use per-joint threshold check
            if self.position_thresholds is None:
                per_joint_ok = all(abs(e) < self.position_threshold for e in errors)
            else:
                per_joint_ok = all(abs(errors[i]) < self.position_thresholds[i] for i in range(3))

            if per_joint_ok:
                if self.stabilization_start_time is None:
                    self.stabilization_start_time = time.time()
                elif time.time() - self.stabilization_start_time >= self.min_stabilization_time:
                    self.get_logger().info('✓ START POSITION REACHED AND STABILIZED. Shutting down.')
                    # hold briefly then shutdown
                    self.create_timer(1.0, self._finish_and_shutdown)
                    self.phase = 'DONE'
            else:
                self.stabilization_start_time = None

            return

    def _publish_torques(self, torques: List[float]):
        m1 = Float64MultiArray(); m1.data = [torques[0]]
        m2 = Float64MultiArray(); m2.data = [torques[1]]
        m3 = Float64MultiArray(); m3.data = [torques[2]]
        self.pub1.publish(m1)
        self.pub2.publish(m2)
        self.pub3.publish(m3)
        
        # Store torques for logging
        self.last_torques = torques.copy()
        
        # Log data to CSV
        self.logger_csv.log_data(
            positions=self.current_joint_pos,
            velocities=self.current_joint_vel,
            torques=torques,
            phase=self.phase
        )

    def _finish_and_shutdown(self):
        # Stop logging
        final_file = self.logger_csv.stop_logging()
        self.get_logger().info(f'Logged {self.logger_csv.get_log_count()} data points to: {final_file}')
        
        # send zero torques then shutdown
        zero = Float64MultiArray(); zero.data = [0.0]
        self.pub1.publish(zero); self.pub2.publish(zero); self.pub3.publish(zero)
        self.get_logger().info('MoveJointTrajectory completed and shutting down')
        rclpy.shutdown()


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    start = None
    end = None
    duration = 5.0

    # parse args: start end duration
    if len(argv) >= 1:
        try:
            start = _parse_vec(argv[0])
        except Exception:
            start = None
    if len(argv) >= 2:
        end = _parse_vec(argv[1])
    if len(argv) >= 3:
        try:
            duration = float(argv[2])
        except Exception:
            duration = 5.0

    # require end
    if end is None:
        print('Usage: move_joint_trajectory.py [start] end duration')
        print('Example: python3 move_joint_trajectory.py "0,0,0" "1.0,-0.5,2.0" 5.0')
        return

    rclpy.init()
    node = MoveJointTrajectory(start=start, end=end, duration=duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
