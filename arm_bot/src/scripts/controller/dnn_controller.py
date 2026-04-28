#!/usr/bin/env python3
"""ROS2 controller node that plans a joint-space min-jerk trajectory
using `path_planner.generate_min_jerk_trajectory` and predicts torques
with `dnn_predictor.DNNInferenceEngine`.

Usage: ros2 run ... or run directly with --target 0 45 90 (degrees)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import numpy as np
import argparse, os, math, time, csv

from path_planner import generate_min_jerk_trajectory
from dnn_predictor import DNNInferenceEngine


class DNNTorqueController(Node):
    LOGS_DIR = os.path.join(os.path.dirname(__file__), 'logs')

    def __init__(self, target_rad, delan, gru, scaler, dt=0.01):
        super().__init__('dnn_torque_controller')
        self.target = np.array(target_rad)
        self.dt = dt
        self.kp = np.array([5.0, 20.0, 10.0])
        self.kd = np.array([1.0, 3.0, 2.0])
        self.torque_limits = np.array([100.0, 100.0, 60.0])

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
        self.joint_states_received = False

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

        # logging (buffer in memory; write on shutdown)
        os.makedirs(self.LOGS_DIR, exist_ok=True)
        self.log_rows = []
        self.log_header = ['t', 'q_des_1','q_des_2','q_des_3','qd_des_1','qd_des_2','qd_des_3',
                           'qdd_des_1','qdd_des_2','qdd_des_3','q_act_1','q_act_2','q_act_3',
                           'qd_act_1','qd_act_2','qd_act_3','tau_delan_1','tau_delan_2','tau_delan_3',
                           'tau_dnn_1','tau_dnn_2','tau_dnn_3','tau_fb_1','tau_fb_2','tau_fb_3','tau_total_1','tau_total_2','tau_total_3','gru_active']

        self.msg1 = Float64MultiArray(); self.msg2 = Float64MultiArray(); self.msg3 = Float64MultiArray()

    def joint_cb(self, msg: JointState):
        try:
            i1 = msg.name.index('joint_1')
            i2 = msg.name.index('joint_2')
            i3 = msg.name.index('joint_3')
            self.current_joint_pos = np.array([msg.position[i1], msg.position[i2], msg.position[i3]])
            if len(msg.velocity) >= 3:
                self.current_joint_vel = np.array([msg.velocity[i1], msg.velocity[i2], msg.velocity[i3]])
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
        # compute time horizon heuristics based on delta
        dq = np.abs(self.target - self.current_joint_pos)
        v_max = 2.0; a_max = 7.0
        T_vel = np.max(1.875 * dq / v_max)
        T_acc = np.max(np.sqrt(5.77 * dq / a_max))
        T_min = max(T_vel, T_acc, 0.5)
        # pick a comfortable total time
        T_total = max(T_min, 2.0)
        self.t, self.q, self.qd, self.qdd, self.xyz = generate_min_jerk_trajectory(self.current_joint_pos, self.target, T_total, self.dt)
        self.n_points = len(self.t)
        self.current_idx = 0
        self.publish_expected_path_marker()

        # start timer
        self.timer = self.create_timer(self.dt, self.trajectory_cb)

    def trajectory_cb(self):
        if self.current_idx >= self.n_points:
            # hold at final position (publish zero velocity torques) and stop timer
            if hasattr(self, 'timer'):
                self.timer.cancel()
            return

        q_des = self.q[self.current_idx]
        qd_des = self.qd[self.current_idx]
        qdd_des = self.qdd[self.current_idx]

        try:
            tau_dnn, tau_delan, gru_active = self.dnn.predict(q_des, qd_des, qdd_des)
        except Exception:
            tau_dnn = tau_delan = np.zeros(3); gru_active = False

        e_pos = q_des - self.current_joint_pos
        e_vel = qd_des - self.current_joint_vel
        tau_fb = self.kp * e_pos + self.kd * e_vel
        tau_total = np.clip(tau_dnn + tau_fb, -self.torque_limits, self.torque_limits)

        self.msg1.data = [float(tau_total[0])]; self.msg2.data=[float(tau_total[1])]; self.msg3.data=[float(tau_total[2])]
        self.pub1.publish(self.msg1); self.pub2.publish(self.msg2); self.pub3.publish(self.msg3)

        # buffer log row
        row = [f'{self.t[self.current_idx]:.3f}', *[f'{v:.8f}' for v in q_des], *[f'{v:.8f}' for v in qd_des], *[f'{v:.8f}' for v in qdd_des], *[f'{v:.8f}' for v in self.current_joint_pos], *[f'{v:.8f}' for v in self.current_joint_vel], *[f'{v:.8f}' for v in tau_delan], *[f'{v:.8f}' for v in tau_dnn], *[f'{v:.8f}' for v in tau_fb], *[f'{v:.8f}' for v in tau_total], '1' if gru_active else '0']
        self.log_rows.append(row)

        self.current_idx += 1

    def save_log_on_shutdown(self):
        # write entire buffer to file
        try:
            name = f'traj_log_{int(time.time())}.csv'
            path = os.path.join(self.LOGS_DIR, name)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(self.log_header)
                w.writerows(self.log_rows)
            self.get_logger().info(f'✓ Log saved: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save log: {e}')


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

        node.plan_and_start()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        # on termination, save log and publish zeros
        node.save_log_on_shutdown()
        zero = Float64MultiArray(); zero.data=[0.0]
        node.pub1.publish(zero); node.pub2.publish(zero); node.pub3.publish(zero)
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
