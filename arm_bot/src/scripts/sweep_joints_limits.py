#!/usr/bin/env python3
"""
sweep_joints_limits.py

ROS2 node to sweep each joint individually from its lower limit to upper limit,
hold for 1 second, then return to the lower limit. After each joint completes its
sweep, it is fixed at a specific position while the next joint is actuated.

Joint behavior:
- Joint 1: Sweeps full range, then moves to 0° and locks there
- Joint 2: Sweeps full range (with Joint 1 locked at 0°), then returns to starting position and locks
- Joint 3: Sweeps full range (with Joints 1 & 2 locked at their positions)

Joint limits (from URDF):
- Joint 1: -160° to +160° (±8π/9 rad)
- Joint 2: -225° to +45° (-5π/4 to π/4 rad)
- Joint 3: -45° to +225° (-π/4 to 5π/4 rad)

Timing per joint:
- 4 seconds: sweep from lower to upper limit
- 1 second: hold at upper limit
- 4 seconds: return from upper to lower limit
- 2 seconds: move to fixed position
- 0.5+ seconds: stabilize at fixed position
Total: ~11.5 seconds per joint

Usage:
  python3 sweep_joints_limits.py
  ros2 run arm_bot sweep_joints_limits.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math
from typing import List
from trajectory_logger import TrajectoryLogger


class SweepJointsLimits(Node):
    def __init__(self):
        super().__init__('sweep_joints_limits')

        # Publishers for each joint effort controller
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)

        # Subscriber to joint_states
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Current robot state
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False

        # Joint limits from URDF (radians)
        # Joint 1: ±8π/9, Joint 2: -5π/4 to π/4, Joint 3: -π/4 to 5π/4
        self.joint_limits = [
            (-8*math.pi/9, 8*math.pi/9),      # Joint 1
            (-5*math.pi/4, math.pi/4),         # Joint 2
            (-math.pi/4, 5*math.pi/4)          # Joint 3
        ]

        # Control params (PID gains similar to move_joint_trajectory)
        self.kp = [30.0, 100.0, 50.0]
        self.ki = [0.5, 1.5, 0.9]
        self.kd = [8.0, 25.0, 10.0]
        self.integral_error = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        self.max_torques = [80.0, 150.0, 40.0]
        self.max_integral = [10.0, 25.0, 5.0]

        # Execution state
        # Phases: WAIT_START -> STABILIZING -> SWEEPING_UP -> HOLDING -> SWEEPING_DOWN -> RETURNING_TO_FIXED -> STABILIZING_FIXED -> NEXT_JOINT -> DONE
        self.phase = 'WAIT_START'
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Sweep parameters
        self.sweep_up_duration = 4.0      # seconds to go from lower to upper
        self.hold_duration = 1.0          # seconds to hold at upper limit
        self.sweep_down_duration = 4.0    # seconds to return to lower limit
        
        self.current_joint_index = 0      # which joint we're currently sweeping (0, 1, or 2)
        self.initial_positions = None     # store initial joint positions
        self.target_positions = [0.0, 0.0, 0.0]  # current target for all joints
        self.fixed_positions = [None, None, None]  # positions to keep joints fixed at after their sweep
        
        self.sweep_start_time = None
        self.hold_start_time = None
        
        # Stabilization criteria
        self.position_threshold = math.radians(5)  # 5 degrees
        self.min_stabilization_time = 0.5
        self.stabilization_start_time = None

        # Initialize trajectory logger
        self.logger_csv = TrajectoryLogger(base_name="sweep_joints", output_dir="./logs")
        self.csv_file = self.logger_csv.start_logging()
        self.last_torques = [0.0, 0.0, 0.0]  # Store last commanded torques for logging

        self.get_logger().info('SweepJointsLimits initialized')
        self.get_logger().info(f'Logging to: {self.csv_file} (Test #{self.logger_csv.get_test_number()})')
        self.get_logger().info('Will sweep each joint through its full range:')
        self.get_logger().info(f'  Joint 1: {math.degrees(self.joint_limits[0][0]):.1f}° to {math.degrees(self.joint_limits[0][1]):.1f}°')
        self.get_logger().info(f'  Joint 2: {math.degrees(self.joint_limits[1][0]):.1f}° to {math.degrees(self.joint_limits[1][1]):.1f}°')
        self.get_logger().info(f'  Joint 3: {math.degrees(self.joint_limits[2][0]):.1f}° to {math.degrees(self.joint_limits[2][1]):.1f}°')

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

    def _compute_pid(self, errors):
        # Update integral with anti-windup
        for i in range(3):
            self.integral_error[i] += errors[i] * self.dt
            self.integral_error[i] = max(-self.max_integral[i], min(self.max_integral[i], self.integral_error[i]))

        derivative = [(errors[i] - self.previous_error[i]) / self.dt for i in range(3)]

        torques = [
            self.kp[i] * errors[i] + self.ki[i] * self.integral_error[i] + self.kd[i] * derivative[i]
            for i in range(3)
        ]

        # Saturate
        torques = [max(-self.max_torques[i], min(self.max_torques[i], torques[i])) for i in range(3)]
        self.previous_error = errors.copy()
        return torques

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
            phase=f"{self.phase}_JOINT{self.current_joint_index+1}"
        )

    def control_loop(self):
        if not self.joint_states_received:
            return

        if self.phase == 'WAIT_START':
            # Record initial positions as the starting point
            self.initial_positions = self.current_joint_pos.copy()
            self.target_positions = self.initial_positions.copy()
            self.get_logger().info(f'Initial positions: [{self.initial_positions[0]:.4f}, {self.initial_positions[1]:.4f}, {self.initial_positions[2]:.4f}]')
            self.get_logger().info('Stabilizing at initial position...')
            self.phase = 'STABILIZING'
            self.stabilization_start_time = None
            self.integral_error = [0.0, 0.0, 0.0]
            self.previous_error = [0.0, 0.0, 0.0]
            return

        if self.phase == 'STABILIZING':
            # Hold at current target position (respecting fixed positions)
            for i in range(3):
                if self.fixed_positions[i] is not None:
                    self.target_positions[i] = self.fixed_positions[i]
                    
            errors = [self.target_positions[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            # Check if stabilized
            per_joint_ok = all(abs(e) < self.position_threshold for e in errors)
            
            if per_joint_ok:
                if self.stabilization_start_time is None:
                    self.stabilization_start_time = time.time()
                elif time.time() - self.stabilization_start_time >= self.min_stabilization_time:
                    if self.current_joint_index < 3:
                        self.get_logger().info(f'✓ Stabilized. Starting sweep for Joint {self.current_joint_index + 1}')
                        self.get_logger().info(f'  Sweeping from {math.degrees(self.joint_limits[self.current_joint_index][0]):.1f}° to {math.degrees(self.joint_limits[self.current_joint_index][1]):.1f}°')
                        # Show which joints are fixed
                        fixed_info = []
                        for i in range(self.current_joint_index):
                            fixed_info.append(f'Joint {i+1} fixed at {math.degrees(self.fixed_positions[i]):.1f}°')
                        if fixed_info:
                            self.get_logger().info(f'  {", ".join(fixed_info)}')
                        self.phase = 'SWEEPING_UP'
                        self.sweep_start_time = time.time()
                        # Reset integrator for sweep
                        self.integral_error = [0.0, 0.0, 0.0]
                        self.previous_error = [0.0, 0.0, 0.0]
                    else:
                        self.get_logger().info('✓ All joints swept! Shutting down.')
                        self.create_timer(1.0, self._finish_and_shutdown)
                        self.phase = 'DONE'
            else:
                self.stabilization_start_time = None

            return

        if self.phase == 'SWEEPING_UP':
            # Sweep current joint from lower limit to upper limit
            elapsed = time.time() - self.sweep_start_time
            s = min(1.0, elapsed / self.sweep_up_duration)
            
            lower_limit = self.joint_limits[self.current_joint_index][0]
            upper_limit = self.joint_limits[self.current_joint_index][1]
            
            # Interpolate the current joint, keep others at their fixed position (or initial if not fixed yet)
            for i in range(3):
                if i == self.current_joint_index:
                    self.target_positions[i] = lower_limit + s * (upper_limit - lower_limit)
                elif self.fixed_positions[i] is not None:
                    self.target_positions[i] = self.fixed_positions[i]
                else:
                    self.target_positions[i] = self.initial_positions[i]
            
            errors = [self.target_positions[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            # Log periodically
            if int(elapsed / 0.5) != int((elapsed - self.dt) / 0.5):
                self.get_logger().info(f'Joint {self.current_joint_index + 1} SWEEPING UP... t={elapsed:.2f}/{self.sweep_up_duration:.2f}s | pos={math.degrees(self.current_joint_pos[self.current_joint_index]):.1f}°')

            if s >= 1.0:
                self.get_logger().info(f'✓ Joint {self.current_joint_index + 1} reached upper limit. Holding for {self.hold_duration}s...')
                self.phase = 'HOLDING'
                self.hold_start_time = time.time()
                # Reset integrator
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]

            return

        if self.phase == 'HOLDING':
            # Hold at upper limit
            errors = [self.target_positions[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            elapsed_hold = time.time() - self.hold_start_time
            if elapsed_hold >= self.hold_duration:
                self.get_logger().info(f'✓ Hold complete. Returning Joint {self.current_joint_index + 1} to lower limit...')
                self.phase = 'SWEEPING_DOWN'
                self.sweep_start_time = time.time()
                # Reset integrator
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]

            return

        if self.phase == 'SWEEPING_DOWN':
            # Sweep current joint from upper limit back to lower limit
            elapsed = time.time() - self.sweep_start_time
            s = min(1.0, elapsed / self.sweep_down_duration)
            
            lower_limit = self.joint_limits[self.current_joint_index][0]
            upper_limit = self.joint_limits[self.current_joint_index][1]
            
            # Interpolate the current joint, keep others at their fixed position (or initial if not fixed yet)
            for i in range(3):
                if i == self.current_joint_index:
                    self.target_positions[i] = upper_limit + s * (lower_limit - upper_limit)
                elif self.fixed_positions[i] is not None:
                    self.target_positions[i] = self.fixed_positions[i]
                else:
                    self.target_positions[i] = self.initial_positions[i]
            
            errors = [self.target_positions[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            # Log periodically
            if int(elapsed / 0.5) != int((elapsed - self.dt) / 0.5):
                self.get_logger().info(f'Joint {self.current_joint_index + 1} SWEEPING DOWN... t={elapsed:.2f}/{self.sweep_down_duration:.2f}s | pos={math.degrees(self.current_joint_pos[self.current_joint_index]):.1f}°')

            if s >= 1.0:
                self.get_logger().info(f'✓ Joint {self.current_joint_index + 1} returned to lower limit.')
                
                # Determine the fixed position for this joint
                if self.current_joint_index == 0:
                    # Joint 1: fix at 0 position
                    fixed_pos = 0.0
                    self.get_logger().info(f'Moving Joint 1 to 0° position...')
                    self.phase = 'RETURNING_TO_FIXED'
                    self.sweep_start_time = time.time()
                    self.target_fixed_position = fixed_pos
                elif self.current_joint_index == 1:
                    # Joint 2: fix at starting position
                    fixed_pos = self.initial_positions[self.current_joint_index]
                    self.get_logger().info(f'Keeping Joint 2 at starting position ({math.degrees(fixed_pos):.1f}°)...')
                    self.phase = 'RETURNING_TO_FIXED'
                    self.sweep_start_time = time.time()
                    self.target_fixed_position = fixed_pos
                else:
                    # Joint 3: already at lower limit, just stabilize there
                    fixed_pos = self.joint_limits[self.current_joint_index][0]
                    self.get_logger().info(f'Stabilizing Joint 3 at lower limit ({math.degrees(fixed_pos):.1f}°)...')
                    self.target_fixed_position = fixed_pos
                    self.target_positions[self.current_joint_index] = fixed_pos
                    self.phase = 'STABILIZING_FIXED'
                    self.stabilization_start_time = None
                
                # Reset integrator
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]

            return

        if self.phase == 'RETURNING_TO_FIXED':
            # Move the just-swept joint to its fixed position
            elapsed = time.time() - self.sweep_start_time
            s = min(1.0, elapsed / 2.0)  # 2 seconds to reach fixed position
            
            lower_limit = self.joint_limits[self.current_joint_index][0]
            
            # Interpolate current joint to fixed position, keep others where they are
            for i in range(3):
                if i == self.current_joint_index:
                    self.target_positions[i] = lower_limit + s * (self.target_fixed_position - lower_limit)
                elif self.fixed_positions[i] is not None:
                    self.target_positions[i] = self.fixed_positions[i]
                else:
                    self.target_positions[i] = self.initial_positions[i]
            
            errors = [self.target_positions[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            if s >= 1.0:
                self.get_logger().info(f'✓ Joint {self.current_joint_index + 1} reached fixed position.')
                self.phase = 'STABILIZING_FIXED'
                self.stabilization_start_time = None
                # Reset integrator
                self.integral_error = [0.0, 0.0, 0.0]
                self.previous_error = [0.0, 0.0, 0.0]

            return

        if self.phase == 'STABILIZING_FIXED':
            # Stabilize at the fixed position before moving to next joint
            errors = [self.target_positions[i] - self.current_joint_pos[i] for i in range(3)]
            torques = self._compute_pid(errors)
            self._publish_torques(torques)

            per_joint_ok = all(abs(e) < self.position_threshold for e in errors)
            
            if per_joint_ok:
                if self.stabilization_start_time is None:
                    self.stabilization_start_time = time.time()
                elif time.time() - self.stabilization_start_time >= self.min_stabilization_time:
                    # Lock this joint at its fixed position
                    self.fixed_positions[self.current_joint_index] = self.target_fixed_position
                    self.get_logger().info(f'✓ Joint {self.current_joint_index + 1} stabilized and locked at {math.degrees(self.target_fixed_position):.1f}°')
                    
                    # Move to next joint
                    self.current_joint_index += 1
                    if self.current_joint_index < 3:
                        self.get_logger().info(f'Preparing Joint {self.current_joint_index + 1}...')
                        self.phase = 'STABILIZING'
                        self.stabilization_start_time = None
                    else:
                        self.get_logger().info('All joints completed! Stabilizing at final positions...')
                        self.phase = 'STABILIZING'
                        self.stabilization_start_time = None
                    
                    # Reset integrator
                    self.integral_error = [0.0, 0.0, 0.0]
                    self.previous_error = [0.0, 0.0, 0.0]
            else:
                self.stabilization_start_time = None

            return

    def _finish_and_shutdown(self):
        # Stop logging
        final_file = self.logger_csv.stop_logging()
        self.get_logger().info(f'Logged {self.logger_csv.get_log_count()} data points to: {final_file}')
        
        # Send zero torques then shutdown
        zero = Float64MultiArray(); zero.data = [0.0]
        self.pub1.publish(zero)
        self.pub2.publish(zero)
        self.pub3.publish(zero)
        self.get_logger().info('SweepJointsLimits completed and shutting down')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SweepJointsLimits()
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
