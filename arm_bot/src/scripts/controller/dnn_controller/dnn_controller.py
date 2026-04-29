#!/usr/bin/env python3
"""ROS2 controller node that plans a joint-space min-jerk trajectory
using `path_planner.generate_min_jerk_trajectory` and predicts torques
with `dnn_predictor.DNNInferenceEngine`.

Usage: ros2 run ... or run directly with --target 0 45 90 (degrees)
"""
import sys
import os

# Add current directory to path to allow direct execution
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import numpy as np
import argparse, math, time, csv

try:
    from .path_planner import generate_min_jerk_trajectory
    from .dnn_predictor import DNNInferenceEngine
except ImportError:
    # Fallback for direct execution
    from path_planner import generate_min_jerk_trajectory
    from dnn_predictor import DNNInferenceEngine


class DNNTorqueController(Node):
    LOGS_DIR = os.path.join(os.path.dirname(__file__), 'logs')

    def __init__(self, target_rad, delan, gru, scaler, dt=0.01):
        super().__init__('dnn_torque_controller')
        self.target = np.array(target_rad)
        self.dt = dt
        
        # Trajectory execution gains (Kp, Kd, Ki)
        self.kp = np.array([5.0, 20.0, 10.0])
        self.kd = np.array([1.0, 3.0, 2.0])
        self.ki = np.array([0.05, 0.2, 0.1])
        
        # Stabilization gains (higher for reaching start position)
        self.kp_stab = np.array([50.0, 200.0, 150.0])
        self.ki_stab = np.array([5.0, 25.0, 20.0])
        self.kd_stab = np.array([12.0, 35.0, 10.0])
        
        self.torque_limits = np.array([2.0, 45.0, 10.0])
        self.vel_filter_alpha = 0.25  # velocity low-pass filter

        # pubs
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # subs
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # state
        self.current_joint_pos = np.zeros(3)
        self.current_joint_vel = np.zeros(3)
        self.filtered_joint_vel = np.zeros(3)
        self.joint_states_received = False
        self.traj_integral_error = np.zeros(3)

        # planner + predictor
        self.dnn = DNNInferenceEngine(delan, gru, scaler)

        # trajectory will be generated once joint state received
        self.t = None
        self.q = None
        self.qd = None
        self.qdd = None
        self.xyz = None
        self.n_points = 0
        self.current_idx = 0

        # logging (row-wise format; write on shutdown)
        os.makedirs(self.LOGS_DIR, exist_ok=True)
        # expected trajectory log
        self.log_exp_t = []; self.log_exp_dp1 = []; self.log_exp_dp2 = []; self.log_exp_dp3 = []
        self.log_exp_dv1 = []; self.log_exp_dv2 = []; self.log_exp_dv3 = []
        self.log_exp_da1 = []; self.log_exp_da2 = []; self.log_exp_da3 = []
        self.log_exp_x = []; self.log_exp_y = []; self.log_exp_z = []

        # actual movement log with torque breakdown
        self.log_act_t = []
        self.log_act_q1 = []; self.log_act_q2 = []; self.log_act_q3 = []
        self.log_act_qd1 = []; self.log_act_qd2 = []; self.log_act_qd3 = []
        self.log_act_dp1 = []; self.log_act_dp2 = []; self.log_act_dp3 = []
        self.log_act_dv1 = []; self.log_act_dv2 = []; self.log_act_dv3 = []
        self.log_act_da1 = []; self.log_act_da2 = []; self.log_act_da3 = []
        self.log_tau_delan1 = []; self.log_tau_delan2 = []; self.log_tau_delan3 = []
        self.log_tau_dnn1 = []; self.log_tau_dnn2 = []; self.log_tau_dnn3 = []
        self.log_tau_pid1 = []; self.log_tau_pid2 = []; self.log_tau_pid3 = []
        self.log_tau_total1 = []; self.log_tau_total2 = []; self.log_tau_total3 = []
        self.log_err1 = []; self.log_err2 = []; self.log_err3 = []
        self.log_vel_err1 = []; self.log_vel_err2 = []; self.log_vel_err3 = []

        self.msg1 = Float64MultiArray(); self.msg2 = Float64MultiArray(); self.msg3 = Float64MultiArray()

    def joint_cb(self, msg: JointState):
        try:
            i1 = msg.name.index('joint_1')
            i2 = msg.name.index('joint_2')
            i3 = msg.name.index('joint_3')
            self.current_joint_pos = np.array([msg.position[i1], msg.position[i2], msg.position[i3]])
            if len(msg.velocity) >= 3:
                raw_vel = np.array([msg.velocity[i1], msg.velocity[i2], msg.velocity[i3]])
                # Low-pass filter velocity
                a = self.vel_filter_alpha
                self.filtered_joint_vel = (1.0 - a) * self.filtered_joint_vel + a * raw_vel
            self.joint_states_received = True
        except ValueError:
            pass

    def publish_expected_path_marker(self):
        if self.xyz is None:
            return
        mk = Marker()
        mk.header.frame_id = 'world'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'expected_path'; mk.id = 0
        mk.type = Marker.LINE_STRIP; mk.action = Marker.ADD
        mk.scale.x = 0.012
        mk.color.r = 0.0; mk.color.g = 1.0; mk.color.b = 1.0; mk.color.a = 0.95
        for p in self.xyz:
            pt = Point(); pt.x = float(p[0]); pt.y = float(p[1]); pt.z = float(p[2])
            mk.points.append(pt)
        self.marker_pub.publish(mk)

    def plan_and_start(self):
        """Plan trajectory and start execution directly."""
        dq = np.abs(self.target - self.current_joint_pos)
        v_max = 2.0; a_max = 7.0
        T_vel = np.max(1.875 * dq / v_max)
        T_acc = np.max(np.sqrt(5.77 * dq / a_max))
        T_min = max(T_vel, T_acc, 0.5)
        # Use longer, more conservative trajectory (min 3.0s)
        T_total = max(T_min * 1.5, 3.0)
        
        self.get_logger().info(f'Planning trajectory: dq={np.degrees(dq).tolist()} deg, T_total={T_total:.2f}s')
        self.t, self.q, self.qd, self.qdd, self.xyz = generate_min_jerk_trajectory(
            self.current_joint_pos, self.target, T_total, self.dt)
        self.n_points = len(self.t)
        self.current_idx = 0
        self._log_expected_trajectory()
        self.publish_expected_path_marker()
        
        # Start trajectory execution directly
        self.start_trajectory_execution()

    def _log_expected_trajectory(self):
        if self.t is None or self.q is None or self.qd is None or self.qdd is None or self.xyz is None:
            return

        self.log_exp_t = [f'{float(v):.3f}' for v in self.t]
        self.log_exp_dp1 = [f'{float(v):.8f}' for v in self.q[:, 0]]
        self.log_exp_dp2 = [f'{float(v):.8f}' for v in self.q[:, 1]]
        self.log_exp_dp3 = [f'{float(v):.8f}' for v in self.q[:, 2]]
        self.log_exp_dv1 = [f'{float(v):.8f}' for v in self.qd[:, 0]]
        self.log_exp_dv2 = [f'{float(v):.8f}' for v in self.qd[:, 1]]
        self.log_exp_dv3 = [f'{float(v):.8f}' for v in self.qd[:, 2]]
        self.log_exp_da1 = [f'{float(v):.8f}' for v in self.qdd[:, 0]]
        self.log_exp_da2 = [f'{float(v):.8f}' for v in self.qdd[:, 1]]
        self.log_exp_da3 = [f'{float(v):.8f}' for v in self.qdd[:, 2]]
        self.log_exp_x = [f'{float(v):.8f}' for v in self.xyz[:, 0]]
        self.log_exp_y = [f'{float(v):.8f}' for v in self.xyz[:, 1]]
        self.log_exp_z = [f'{float(v):.8f}' for v in self.xyz[:, 2]]

    def trajectory_cb(self):
        if self.current_idx >= self.n_points:
            # hold at final position and stop timer
            if hasattr(self, 'timer'):
                self.timer.cancel()
            return

        q_des = self.q[self.current_idx]
        qd_des = self.qd[self.current_idx]
        qdd_des = self.qdd[self.current_idx]
        q_act = self.current_joint_pos.copy()
        qd_act = self.filtered_joint_vel.copy()

        try:
            tau_dnn, tau_delan, gru_active = self.dnn.predict(q_des, qd_des, qdd_des)
        except Exception:
            tau_dnn = tau_delan = np.zeros(3); gru_active = False

        e_pos = q_des - self.current_joint_pos
        e_vel = qd_des - self.filtered_joint_vel
        
        # PD + I feedback control
        self.traj_integral_error += e_pos * self.dt
        self.traj_integral_error = np.clip(
            self.traj_integral_error,
            -np.array([0.3, 0.5, 0.5]), np.array([0.3, 0.5, 0.5]))
        
        tau_fb = self.kp * e_pos + self.kd * e_vel + self.ki * self.traj_integral_error
        tau_total = np.clip(tau_dnn + tau_fb, -self.torque_limits, self.torque_limits)

        self.msg1.data = [float(tau_total[0])]; self.msg2.data=[float(tau_total[1])]; self.msg3.data=[float(tau_total[2])]
        self.pub1.publish(self.msg1); self.pub2.publish(self.msg2); self.pub3.publish(self.msg3)

        # buffer actual movement log values (row-wise format)
        self.log_act_t.append(f'{self.t[self.current_idx]:.3f}')
        self.log_act_q1.append(f'{q_act[0]:.8f}'); self.log_act_q2.append(f'{q_act[1]:.8f}'); self.log_act_q3.append(f'{q_act[2]:.8f}')
        self.log_act_qd1.append(f'{qd_act[0]:.8f}'); self.log_act_qd2.append(f'{qd_act[1]:.8f}'); self.log_act_qd3.append(f'{qd_act[2]:.8f}')
        self.log_act_dp1.append(f'{q_des[0]:.8f}'); self.log_act_dp2.append(f'{q_des[1]:.8f}'); self.log_act_dp3.append(f'{q_des[2]:.8f}')
        self.log_act_dv1.append(f'{qd_des[0]:.8f}'); self.log_act_dv2.append(f'{qd_des[1]:.8f}'); self.log_act_dv3.append(f'{qd_des[2]:.8f}')
        self.log_act_da1.append(f'{qdd_des[0]:.8f}'); self.log_act_da2.append(f'{qdd_des[1]:.8f}'); self.log_act_da3.append(f'{qdd_des[2]:.8f}')
        self.log_tau_delan1.append(f'{tau_delan[0]:.8f}'); self.log_tau_delan2.append(f'{tau_delan[1]:.8f}'); self.log_tau_delan3.append(f'{tau_delan[2]:.8f}')
        self.log_tau_dnn1.append(f'{tau_dnn[0]:.8f}'); self.log_tau_dnn2.append(f'{tau_dnn[1]:.8f}'); self.log_tau_dnn3.append(f'{tau_dnn[2]:.8f}')
        self.log_tau_pid1.append(f'{tau_fb[0]:.8f}'); self.log_tau_pid2.append(f'{tau_fb[1]:.8f}'); self.log_tau_pid3.append(f'{tau_fb[2]:.8f}')
        self.log_tau_total1.append(f'{tau_total[0]:.8f}'); self.log_tau_total2.append(f'{tau_total[1]:.8f}'); self.log_tau_total3.append(f'{tau_total[2]:.8f}')
        self.log_err1.append(f'{e_pos[0]:.8f}'); self.log_err2.append(f'{e_pos[1]:.8f}'); self.log_err3.append(f'{e_pos[2]:.8f}')
        self.log_vel_err1.append(f'{e_vel[0]:.8f}'); self.log_vel_err2.append(f'{e_vel[1]:.8f}'); self.log_vel_err3.append(f'{e_vel[2]:.8f}')

        if self.current_idx % 50 == 0 and self.current_idx < self.n_points:
            max_err_deg = math.degrees(np.max(np.abs(e_pos)))
            self.get_logger().info(
                f't={self.t[self.current_idx]:.1f}s | '
                f'idx={self.current_idx}/{self.n_points} | '
                f'τ=[{tau_total[0]:.1f},{tau_total[1]:.1f},{tau_total[2]:.1f}] Nm | '
                f'err={max_err_deg:.2f}°')

        self.current_idx += 1

    def start_trajectory_execution(self):
        """Start trajectory execution with DNN + PD+I feedback."""
        self.get_logger().info('=' * 80)
        self.get_logger().info('TRAJECTORY EXECUTION (DNN + PD+I feedback)')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'{self.n_points} steps | dt={self.dt*1000:.1f}ms')
        
        self.current_idx = 0
        self.traj_integral_error = np.zeros(3)
        self.timer = self.create_timer(self.dt, self.trajectory_cb)

    def save_expected_trajectory_log(self):
        try:
            name = f'expected_trajectory_{int(time.time())}.csv'
            path = os.path.join(self.LOGS_DIR, name)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t'] + self.log_exp_t)
                w.writerow(['dp1'] + self.log_exp_dp1); w.writerow(['dp2'] + self.log_exp_dp2); w.writerow(['dp3'] + self.log_exp_dp3)
                w.writerow(['dv1'] + self.log_exp_dv1); w.writerow(['dv2'] + self.log_exp_dv2); w.writerow(['dv3'] + self.log_exp_dv3)
                w.writerow(['da1'] + self.log_exp_da1); w.writerow(['da2'] + self.log_exp_da2); w.writerow(['da3'] + self.log_exp_da3)
                w.writerow(['x'] + self.log_exp_x); w.writerow(['y'] + self.log_exp_y); w.writerow(['z'] + self.log_exp_z)
            self.get_logger().info(f'✓ Expected trajectory saved: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save expected trajectory log: {e}')

    def save_actual_movement_log(self):
        try:
            name = f'actual_movement_{int(time.time())}.csv'
            path = os.path.join(self.LOGS_DIR, name)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t'] + self.log_act_t)
                w.writerow(['q1'] + self.log_act_q1); w.writerow(['q2'] + self.log_act_q2); w.writerow(['q3'] + self.log_act_q3)
                w.writerow(['qd1'] + self.log_act_qd1); w.writerow(['qd2'] + self.log_act_qd2); w.writerow(['qd3'] + self.log_act_qd3)
                w.writerow(['dp1'] + self.log_act_dp1); w.writerow(['dp2'] + self.log_act_dp2); w.writerow(['dp3'] + self.log_act_dp3)
                w.writerow(['dv1'] + self.log_act_dv1); w.writerow(['dv2'] + self.log_act_dv2); w.writerow(['dv3'] + self.log_act_dv3)
                w.writerow(['da1'] + self.log_act_da1); w.writerow(['da2'] + self.log_act_da2); w.writerow(['da3'] + self.log_act_da3)
                w.writerow(['tau_delan1'] + self.log_tau_delan1); w.writerow(['tau_delan2'] + self.log_tau_delan2); w.writerow(['tau_delan3'] + self.log_tau_delan3)
                w.writerow(['tau_dnn1'] + self.log_tau_dnn1); w.writerow(['tau_dnn2'] + self.log_tau_dnn2); w.writerow(['tau_dnn3'] + self.log_tau_dnn3)
                w.writerow(['tau_pid1'] + self.log_tau_pid1); w.writerow(['tau_pid2'] + self.log_tau_pid2); w.writerow(['tau_pid3'] + self.log_tau_pid3)
                w.writerow(['tau_total1'] + self.log_tau_total1); w.writerow(['tau_total2'] + self.log_tau_total2); w.writerow(['tau_total3'] + self.log_tau_total3)
                w.writerow(['err1'] + self.log_err1); w.writerow(['err2'] + self.log_err2); w.writerow(['err3'] + self.log_err3)
                w.writerow(['vel_err1'] + self.log_vel_err1); w.writerow(['vel_err2'] + self.log_vel_err2); w.writerow(['vel_err3'] + self.log_vel_err3)
            self.get_logger().info(f'✓ Actual movement saved: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save actual movement log: {e}')

    def save_logs_on_shutdown(self):
        self.save_expected_trajectory_log()
        self.save_actual_movement_log()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', type=float, nargs=3, required=True, help='Target joint angles (deg)')
    parser.add_argument('--delan-model', type=str, required=True)
    parser.add_argument('--gru-model', type=str, required=True)
    parser.add_argument('--scaler', type=str, required=True)
    parser.add_argument('--dt', type=float, default=0.01)
    args = parser.parse_args()

    rclpy.init()
    target_rad = np.deg2rad(np.array(args.target))
    node = DNNTorqueController(target_rad, args.delan_model, args.gru_model, args.scaler, dt=args.dt)
    try:
        # wait for joint states
        node.get_logger().info('Waiting for joint states...')
        while not node.joint_states_received and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.joint_states_received:
            node.get_logger().error('No joint states received, exiting')
            return

        node.get_logger().info(
            f'Current position: {np.degrees(node.current_joint_pos).tolist()} deg')
        
        # Plan and start (Phase 1: stabilization or skip, Phase 2: execution)
        node.plan_and_start()
        
        # Main spin loop: trajectory execution
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            # Check if trajectory execution is done
            if hasattr(node, 'timer') and node.current_idx >= node.n_points:
                break
                
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        # on termination, save log and publish zeros
        node.save_logs_on_shutdown()
        try:
            zero = Float64MultiArray(); zero.data=[0.0]
            node.pub1.publish(zero); node.pub2.publish(zero); node.pub3.publish(zero)
        except Exception:
            pass
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
