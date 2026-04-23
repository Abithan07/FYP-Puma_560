#!/usr/bin/env python3
"""
Optimized Torque Publisher - 100Hz Operation
Publishes feedforward inverse-dynamics torques from CSV and adds PID feedback
during full trajectory tracking after stabilization.
"""

import argparse
import csv
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class TorquePublisher(Node):
    def __init__(self, csv_path=None):
        super().__init__("torque_publisher")

        # Publishers for effort commands
        self.pub1 = self.create_publisher(Float64MultiArray, "/joint_1_controller/commands", 10)
        self.pub2 = self.create_publisher(Float64MultiArray, "/joint_2_controller/commands", 10)
        self.pub3 = self.create_publisher(Float64MultiArray, "/joint_3_controller/commands", 10)

        # Subscriber for feedback
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        # CSV path
        if csv_path is None:
            self.csv_path = os.path.expanduser(
                "/data/ros2/ros2_ws2/arm_bot/src/scripts/script_resources/path_021_joint_states_modified.csv"
            )
        else:
            self.csv_path = os.path.expanduser(csv_path)

        # Load all trajectory data once
        self.load_trajectory_data()

        # Pre-allocate messages
        self.msg1 = Float64MultiArray()
        self.msg2 = Float64MultiArray()
        self.msg3 = Float64MultiArray()

        # Robot state
        self.current_idx = 0
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False

        # Execution state
        self.trajectory_active = False
        self.trajectory_timer = None
        self.stabilization_timer = None
        self.stabilization_complete = False
        self.stabilization_iterations = 0

        # Phase-1 stabilization PID integral
        self.integral_error = [0.0, 0.0, 0.0]

        # Phase-2 trajectory tracking PID state
        self.traj_integral_error = [0.0, 0.0, 0.0]
        self.traj_kp = [30.0, 80.0, 60.0]
        self.traj_ki = [2.0, 8.0, 6.0]
        self.traj_kd = [4.0, 12.0, 8.0]
        self.traj_int_limit = [0.4, 0.8, 0.8]

        # Torque saturation
        self.max_torques = [100.0, 100.0, 50.0]

        # Logger subprocess/service clients
        self.logger_process = None
        self.logger_start_client = None
        self.logger_stop_client = None

        self.get_logger().info("=" * 70)
        self.get_logger().info("OPTIMIZED TORQUE PUBLISHER - FEEDFORWARD + PID TRACKING")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Loaded {len(self.time_data)} trajectory points")
        self.get_logger().info(f"Trajectory duration: {self.time_data[-1]:.2f}s")
        self.get_logger().info(f"Control frequency: 100 Hz (dt={self.dt:.4f}s)")
        self.get_logger().info(
            f"Initial target: [{self.dp1[0]:.4f}, {self.dp2[0]:.4f}, {self.dp3[0]:.4f}] rad"
        )

    def load_trajectory_data(self):
        data = {}
        with open(self.csv_path, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                key = row[0]
                values = [float(v) for v in row[1:]]
                data[key] = values

        # Required keys
        required = ["t", "dp1", "dp2", "dp3", "tau1", "tau2", "tau3"]
        missing = [k for k in required if k not in data]
        if missing:
            raise RuntimeError(f"Missing required CSV keys: {missing}")

        self.time_data = data["t"]
        self.dp1 = data["dp1"]
        self.dp2 = data["dp2"]
        self.dp3 = data["dp3"]
        self.tau1 = data["tau1"]
        self.tau2 = data["tau2"]
        self.tau3 = data["tau3"]

        n = len(self.time_data)
        if not (len(self.dp1) == len(self.dp2) == len(self.dp3) == len(self.tau1) == len(self.tau2) == len(self.tau3) == n):
            raise RuntimeError("CSV arrays have mismatched lengths")

        self.dt = self.time_data[1] - self.time_data[0] if n > 1 else 0.01
        if self.dt <= 0.0:
            self.dt = 0.01

        # Optional desired velocity keys; fallback to numerical derivative
        if "dq1" in data and "dq2" in data and "dq3" in data:
            self.dq1 = data["dq1"]
            self.dq2 = data["dq2"]
            self.dq3 = data["dq3"]
            if not (len(self.dq1) == len(self.dq2) == len(self.dq3) == n):
                raise RuntimeError("dq arrays exist but lengths do not match trajectory length")
        else:
            self.dq1 = self._compute_vel_from_pos(self.dp1, self.time_data)
            self.dq2 = self._compute_vel_from_pos(self.dp2, self.time_data)
            self.dq3 = self._compute_vel_from_pos(self.dp3, self.time_data)

    def _compute_vel_from_pos(self, pos, t):
        n = len(pos)
        if n <= 1:
            return [0.0] * n

        vel = [0.0] * n
        for i in range(1, n):
            dt_i = t[i] - t[i - 1]
            if dt_i <= 1e-9:
                dt_i = self.dt
            vel[i] = (pos[i] - pos[i - 1]) / dt_i
        vel[0] = vel[1]
        return vel

    def joint_state_callback(self, msg):
        try:
            idx1 = msg.name.index("joint_1")
            idx2 = msg.name.index("joint_2")
            idx3 = msg.name.index("joint_3")

            self.current_joint_pos = [
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3],
            ]

            if len(msg.velocity) > max(idx1, idx2, idx3):
                self.current_joint_vel = [
                    msg.velocity[idx1],
                    msg.velocity[idx2],
                    msg.velocity[idx3],
                ]

            self.joint_states_received = True
        except (ValueError, IndexError):
            pass

    def launch_logger(self):
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            logger_script = os.path.join(script_dir, "continuous_logger_triggered.py")

            self.get_logger().info("Launching triggered logger subprocess...")
            cmd = ["python3", logger_script]
            if self.csv_path:
                cmd.extend(["--dataset-path", self.csv_path])

            self.logger_process = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
            )

            time.sleep(0.1)

            self.logger_start_client = self.create_client(Trigger, "/logger/start")
            self.logger_stop_client = self.create_client(Trigger, "/logger/stop")

            timeout = 5.0
            t0 = time.time()
            while not self.logger_start_client.wait_for_service(timeout_sec=0.1):
                if time.time() - t0 > timeout:
                    self.get_logger().error("Logger start service not available")
                    return False
                rclpy.spin_once(self, timeout_sec=0.01)

            self.get_logger().info("Logger subprocess ready")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to launch logger: {e}")
            return False

    def start_logger(self):
        if not self.logger_start_client:
            self.get_logger().error("Logger client not initialized")
            return False

        request = Trigger.Request()
        future = self.logger_start_client.call_async(request)

        t0 = time.time()
        while not future.done():
            if time.time() - t0 > 1.0:
                self.get_logger().error("Logger start service timeout")
                return False
            rclpy.spin_once(self, timeout_sec=0.01)

        response = future.result()
        if response.success:
            self.get_logger().info("Logger recording started")
        else:
            self.get_logger().error(f"Logger start failed: {response.message}")
        return response.success

    def stop_logger(self):
        if not self.logger_stop_client:
            self.get_logger().error("Logger client not initialized")
            return False

        request = Trigger.Request()
        future = self.logger_stop_client.call_async(request)

        t0 = time.time()
        while not future.done():
            if time.time() - t0 > 1.0:
                self.get_logger().error("Logger stop service timeout")
                return False
            rclpy.spin_once(self, timeout_sec=0.01)

        response = future.result()
        if response.success:
            self.get_logger().info(f"Logger stopped: {response.message}")
        else:
            self.get_logger().error(f"Logger stop failed: {response.message}")
        return response.success

    def shutdown_logger(self):
        if self.logger_process:
            self.logger_process.terminate()
            try:
                self.logger_process.wait(timeout=2.0)
                self.get_logger().info("Logger subprocess terminated")
            except subprocess.TimeoutExpired:
                self.logger_process.kill()
                self.get_logger().warning("Logger subprocess killed after timeout")

    def stabilization_callback(self):
        target_pos = [self.dp1[0], self.dp2[0], self.dp3[0]]

        errors = [target_pos[i] - self.current_joint_pos[i] for i in range(3)]
        max_error = max(abs(e) for e in errors)

        # Very strict convergence threshold from your original code
        if max_error < 0.0000035:
            self.get_logger().info(
                f"Initial position reached. Max error: {max_error * 180.0 / 3.14159:.6f} deg"
            )
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return

        self.stabilization_iterations += 1
        if self.stabilization_iterations >= 12000:
            self.get_logger().error(
                f"Timeout in stabilization. Current max error: {max_error * 180.0 / 3.14159:.6f} deg"
            )
            if self.stabilization_timer:
                self.stabilization_timer.cancel()
            self.stabilization_complete = True
            return

        kp = [50.0, 200.0, 150.0]
        ki = [5.0, 25.0, 20.0]
        kd = [12.0, 35.0, 10.0]

        dt = 0.01
        max_integral = [0.5, 1.0, 1.0]
        for i in range(3):
            self.integral_error[i] += errors[i] * dt
            self.integral_error[i] = max(-max_integral[i], min(max_integral[i], self.integral_error[i]))

        q2 = self.current_joint_pos[1]
        q3 = self.current_joint_pos[2]
        gravity_comp = [
            0.0,
            -44.0 * math.cos(q2),
            -12.0 * math.cos(q2 + q3),
        ]

        torques = [
            kp[i] * errors[i]
            + ki[i] * self.integral_error[i]
            - kd[i] * self.current_joint_vel[i]
            + gravity_comp[i]
            for i in range(3)
        ]

        torques = [
            max(-self.max_torques[i], min(self.max_torques[i], torques[i]))
            for i in range(3)
        ]

        self.msg1.data = [torques[0]]
        self.msg2.data = [torques[1]]
        self.msg3.data = [torques[2]]

        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

        if self.stabilization_iterations % 50 == 0:
            self.get_logger().info(
                "t={:.1f}s | err_deg=[{:.5f}, {:.5f}, {:.5f}] | max={:.5f}".format(
                    self.stabilization_iterations / 100.0,
                    errors[0] * 180.0 / 3.14159,
                    errors[1] * 180.0 / 3.14159,
                    errors[2] * 180.0 / 3.14159,
                    max_error * 180.0 / 3.14159,
                )
            )

    def trajectory_callback(self):
        """
        Full trajectory tracking at 100Hz:
        tau_cmd = tau_ff + Kp*e_pos + Ki*int(e_pos) + Kd*e_vel
        """
        if self.current_idx >= len(self.time_data):
            self.get_logger().info("=" * 70)
            self.get_logger().info("TRAJECTORY EXECUTION COMPLETED")
            self.get_logger().info("=" * 70)
            if self.trajectory_timer:
                self.trajectory_timer.cancel()
            self.trajectory_active = False
            return

        i = self.current_idx

        # Desired states
        qd = [self.dp1[i], self.dp2[i], self.dp3[i]]
        dqd = [self.dq1[i], self.dq2[i], self.dq3[i]]

        # Measured states
        q = self.current_joint_pos
        dq = self.current_joint_vel

        # Tracking errors
        e_pos = [qd[j] - q[j] for j in range(3)]
        e_vel = [dqd[j] - dq[j] for j in range(3)]

        # Integrator with anti-windup
        dt = self.dt if self.dt > 0.0 else 0.01
        for j in range(3):
            self.traj_integral_error[j] += e_pos[j] * dt
            lim = self.traj_int_limit[j]
            if self.traj_integral_error[j] > lim:
                self.traj_integral_error[j] = lim
            elif self.traj_integral_error[j] < -lim:
                self.traj_integral_error[j] = -lim

        # Feedforward torque from CSV
        tau_ff = [self.tau1[i], self.tau2[i], self.tau3[i]]

        # Feedforward + PID feedback
        torques = [0.0, 0.0, 0.0]
        for j in range(3):
            tau = (
                tau_ff[j]
                + self.traj_kp[j] * e_pos[j]
                + self.traj_ki[j] * self.traj_integral_error[j]
                + self.traj_kd[j] * e_vel[j]
            )

            # Saturation
            tau = max(-self.max_torques[j], min(self.max_torques[j], tau))
            torques[j] = tau

        # Publish torques
        self.msg1.data = [torques[0]]
        self.msg2.data = [torques[1]]
        self.msg3.data = [torques[2]]
        self.pub1.publish(self.msg1)
        self.pub2.publish(self.msg2)
        self.pub3.publish(self.msg3)

        self.current_idx += 1

        if self.current_idx % 50 == 0 and self.current_idx < len(self.time_data):
            max_err_deg = max(abs(e) for e in e_pos) * 180.0 / 3.14159
            self.get_logger().info(
                f"t={self.time_data[i]:.2f}s | idx={self.current_idx}/{len(self.time_data)} | "
                f"tau=[{torques[0]:.2f}, {torques[1]:.2f}, {torques[2]:.2f}] Nm | "
                f"max_err={max_err_deg:.3f} deg"
            )

    def run(self):
        self.get_logger().info("Waiting for joint states...")

        while not self.joint_states_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.joint_states_received:
            self.get_logger().error("Failed to receive joint states")
            return

        self.get_logger().info(
            f"Current position: [{self.current_joint_pos[0]:.4f}, "
            f"{self.current_joint_pos[1]:.4f}, {self.current_joint_pos[2]:.4f}] rad"
        )

        # Phase 1: Stabilization
        self.get_logger().info("=" * 70)
        self.get_logger().info("PHASE 1: STABILIZATION")
        self.get_logger().info("=" * 70)
        self.stabilization_iterations = 0
        self.stabilization_complete = False
        self.integral_error = [0.0, 0.0, 0.0]
        self.stabilization_timer = self.create_timer(0.01, self.stabilization_callback)

        while not self.stabilization_complete and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

        if not rclpy.ok():
            return

        # Hold for 1 second
        self.get_logger().info("Holding position for 1 second...")
        hold_iterations = [0]

        def hold_callback():
            hold_iterations[0] += 1
            if hold_iterations[0] >= 100:
                hold_timer.cancel()
                return
            self.stabilization_callback()

        hold_timer = self.create_timer(0.01, hold_callback)
        while hold_iterations[0] < 100 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

        # Launch logger subprocess
        self.get_logger().info("=" * 70)
        self.get_logger().info("Launching data logger...")
        if not self.launch_logger():
            self.get_logger().error("Failed to launch logger, continuing without logging")

        # Phase 2: Full trajectory feedforward + PID tracking
        self.get_logger().info("=" * 70)
        self.get_logger().info("PHASE 2: TRAJECTORY TRACKING (FEEDFORWARD + PID)")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Publishing {len(self.time_data)} commands at 100Hz...")

        time.sleep(0.1)
        if self.logger_start_client:
            self.start_logger()

        time.sleep(0.01)

        self.current_idx = 0
        self.traj_integral_error = [0.0, 0.0, 0.0]
        self.trajectory_active = True
        self.trajectory_timer = self.create_timer(0.01, self.trajectory_callback)

        while self.trajectory_active and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

        time.sleep(0.1)
        if self.logger_stop_client:
            self.stop_logger()

        time.sleep(0.1)
        self.get_logger().info("Torque publisher shutting down.")

def main(args=None):
    parser = argparse.ArgumentParser(description="Torque Publisher with trajectory tracking PID")
    parser.add_argument(
        "--csv-path",
        type=str,
        default=None,
        help="Path to CSV file containing trajectory data",
    )
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = TorquePublisher(csv_path=parsed_args.csv_path)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.shutdown_logger()

        # Zero torques on shutdown
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0]
        node.pub1.publish(zero_msg)
        node.pub2.publish(zero_msg)
        node.pub3.publish(zero_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()