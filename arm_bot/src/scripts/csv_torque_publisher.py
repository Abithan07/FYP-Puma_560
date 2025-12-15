import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import csv
import os
import sys
import time
import math

class CSVTorquePublisher(Node):
    def __init__(self, csv_file=None):
        super().__init__('csv_torque_publisher')
        
        # Use provided csv_file or declare parameter
        if csv_file is None:
            self.declare_parameter('csv_file', 'torque_values.csv')
            csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        
        # Publishers for each joint
        self.pub1 = self.create_publisher(Float64MultiArray, '/joint_1_controller/commands', 10)
        self.pub2 = self.create_publisher(Float64MultiArray, '/joint_2_controller/commands', 10)
        self.pub3 = self.create_publisher(Float64MultiArray, '/joint_3_controller/commands', 10)
        
        # Subscriber to monitor joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Load torque and position data from CSV
        self.torque_data = []
        self.starting_position = None
        self.load_csv_data(csv_file)
        
        # Current joint states
        self.current_joint_pos = [0.0, 0.0, 0.0]
        self.current_joint_vel = [0.0, 0.0, 0.0]
        self.joint_states_received = False
        
        # Stabilization parameters
        self.position_threshold = math.radians(10)  # 5 degrees in radians
        self.is_stabilized = False
        self.stabilization_start_time = None
        self.min_stabilization_time = 1.0  # Must stay stable for 1 second
        
        # PID controller for stabilization
        self.kp = [30.0, 100.0, 15.0]
        self.ki = [0.5, 1.5, 0.2]
        self.kd = [8.0, 25.0, 4.0]
        self.integral_error = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        self.max_torques = [80.0, 150.0, 40.0]
        self.max_integral = [10.0, 25.0, 5.0]
        
        # Execution state
        self.phase = 'STABILIZING'  # STABILIZING, PUBLISHING, or FINAL_STABILIZING
        self.current_row = 0
        self.finished = False
        self.trajectory_start_time = None
        self.final_position = None
        
        # CSV logging setup
        self.csv_file_path = csv_file
        self.output_csv_path = self._generate_output_csv_name(csv_file)
        self.logged_rows = []  # Store logged data before writing
        
        # Timer to run control loop at 100 Hz
        self.timer_period = 0.01
        self.dt = self.timer_period
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info(f'CSV Torque Publisher initialized')
        self.get_logger().info(f'Loaded {len(self.torque_data)} trajectory points from {csv_file}')
        self.get_logger().info(f'Output will be saved to: {self.output_csv_path}')
        self.get_logger().info(f'Starting position: [{self.starting_position[0]:.4f}, {self.starting_position[1]:.4f}, {self.starting_position[2]:.4f}] rad')
        self.get_logger().info(f'Position threshold: {self.position_threshold:.4f} rad ({math.degrees(self.position_threshold):.2f} deg)')
        self.get_logger().info(f'Phase 1: STABILIZING at starting position...')
    
    def _generate_output_csv_name(self, input_csv):
        """Generate output CSV filename with timestamp"""
        import datetime
        base_name = os.path.splitext(input_csv)[0]
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        return f'{base_name}_with_actual_{timestamp}.csv'
    
    def load_csv_data(self, csv_file):
        """Load torque values and starting position from CSV file
        Supports two input formats:
        1. Column-wise (original): Headers with values in rows
        2. Row-wise (new): Variable names in first column, values in columns
        """
        try:
            # Check if file exists
            if not os.path.exists(csv_file):
                self.get_logger().error(f'CSV file not found: {csv_file}')
                return
            
            # First, detect the format by reading the first line
            with open(csv_file, 'r') as file:
                first_line = file.readline().strip()
                # If first line contains column headers like 'dp1,dp2,dp3,...', it's column-wise
                # If it starts with 'tau1' or similar followed by numbers, it's row-wise
                is_row_wise = first_line.split(',')[0].strip().lower() in ['tau1', 'tau2', 'tau3', 'dp1', 'dp2', 'dp3', 't']
            
            if is_row_wise:
                self._load_row_wise_format(csv_file)
            else:
                self._load_column_wise_format(csv_file)
            
            if len(self.torque_data) == 0:
                self.get_logger().error('No valid data found in CSV file')
            if self.starting_position is None:
                self.get_logger().error('Could not extract starting position from CSV')
        
        except Exception as e:
            self.get_logger().error(f'Error loading CSV file: {e}')
    
    def _load_column_wise_format(self, csv_file):
        """Load CSV in column-wise format (headers with values in rows)"""
        with open(csv_file, 'r') as file:
            csv_reader = csv.DictReader(file)
            
            # Read all rows
            for i, row in enumerate(csv_reader):
                try:
                    data_point = {
                        'dp1': float(row['dp1']),
                        'dp2': float(row['dp2']),
                        'dp3': float(row['dp3']),
                        'tau1': float(row['tau1']),
                        'tau2': float(row['tau2']),
                        'tau3': float(row['tau3']),
                        't': float(row['t'])
                    }
                    self.torque_data.append(data_point)
                    
                    # Store first row as starting position
                    if i == 0:
                        self.starting_position = [
                            data_point['dp1'],
                            data_point['dp2'],
                            data_point['dp3']
                        ]
                except (ValueError, KeyError) as e:
                    self.get_logger().warn(f'Skipping invalid row {i}: {e}')
                    continue
    
    def _load_row_wise_format(self, csv_file):
        """Load CSV in row-wise format (variable names in first column, values in columns)
        Input format:
            tau1,value1,value2,value3,...
            tau2,value1,value2,value3,...
            tau3,value1,value2,value3,...
            dp1,value1,value2,value3,...
            dp2,value1,value2,value3,...
            dp3,value1,value2,value3,...
            t,value1,value2,value3,...
        """
        data = {}
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row:
                    key = row[0].strip().lower()  # Get the row name (tau1, tau2, etc.)
                    values = [float(val) for val in row[1:]]  # Get all values in the row
                    data[key] = values
        
        # Check if required data exists
        required_keys = ['tau1', 'tau2', 'tau3']
        for key in required_keys:
            if key not in data:
                self.get_logger().error(f'Missing required key: {key}')
                return
        
        # Generate time data if not present
        if 't' not in data:
            num_points = len(data['tau1'])
            data['t'] = [i * 0.01 for i in range(num_points)]  # 10ms timestep
        
        # Generate position data if not present (default to zero)
        num_points = len(data['tau1'])
        if 'dp1' not in data:
            data['dp1'] = [0.0] * num_points
        if 'dp2' not in data:
            data['dp2'] = [0.0] * num_points
        if 'dp3' not in data:
            data['dp3'] = [0.0] * num_points
        
        # Convert to list of dictionaries
        for i in range(num_points):
            data_point = {
                'dp1': data['dp1'][i],
                'dp2': data['dp2'][i],
                'dp3': data['dp3'][i],
                'tau1': data['tau1'][i],
                'tau2': data['tau2'][i],
                'tau3': data['tau3'][i],
                't': data['t'][i]
            }
            self.torque_data.append(data_point)
        
        # Store first point as starting position
        if len(self.torque_data) > 0:
            self.starting_position = [
                self.torque_data[0]['dp1'],
                self.torque_data[0]['dp2'],
                self.torque_data[0]['dp3']
            ]
    
    def joint_state_callback(self, msg):
        """Monitor current joint positions and velocities"""
        try:
            idx1 = msg.name.index('joint_1')
            idx2 = msg.name.index('joint_2')
            idx3 = msg.name.index('joint_3')
            
            self.current_joint_pos = [
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3]
            ]
            
            if len(msg.velocity) > 0:
                self.current_joint_vel = [
                    msg.velocity[idx1],
                    msg.velocity[idx2],
                    msg.velocity[idx3]
                ]
            
            self.joint_states_received = True
        except (ValueError, IndexError):
            pass
    
    def control_loop(self):
        """Main control loop - handles stabilization, trajectory publishing, and final stabilization"""
        if not self.joint_states_received or len(self.torque_data) == 0:
            return
        
        if self.finished:
            return
        
        if self.phase == 'STABILIZING':
            self.stabilize_phase()
        elif self.phase == 'PUBLISHING':
            self.publish_phase()
        elif self.phase == 'FINAL_STABILIZING':
            self.final_stabilize_phase()
    
    def stabilize_phase(self):
        """PID control to stabilize at starting position"""
        # Calculate position errors
        errors = [self.starting_position[i] - self.current_joint_pos[i] for i in range(3)]
        
        # Update integral with anti-windup
        for i in range(3):
            self.integral_error[i] += errors[i] * self.dt
            self.integral_error[i] = max(-self.max_integral[i], 
                                        min(self.max_integral[i], self.integral_error[i]))
        
        # Calculate derivative
        derivative_error = [(errors[i] - self.previous_error[i]) / self.dt for i in range(3)]
        
        # PID control law
        torques = [
            self.kp[i] * errors[i] + 
            self.ki[i] * self.integral_error[i] + 
            self.kd[i] * derivative_error[i]
            for i in range(3)
        ]
        
        # Saturate torques
        torques = [max(-self.max_torques[i], min(self.max_torques[i], torques[i])) 
                   for i in range(3)]
        
        # Publish torques
        msg1 = Float64MultiArray()
        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()
        
        msg1.data = [torques[0]]
        msg2.data = [torques[1]]
        msg3.data = [torques[2]]
        
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        
        # Update previous error
        self.previous_error = errors.copy()
        
        # Check if stabilized
        max_error = max(abs(e) for e in errors)
        if max_error < self.position_threshold:
            if self.stabilization_start_time is None:
                self.stabilization_start_time = time.time()
            elif time.time() - self.stabilization_start_time >= self.min_stabilization_time:
                if not self.is_stabilized:
                    self.is_stabilized = True
                    self.get_logger().info('='*60)
                    self.get_logger().info(f'✓ STABILIZATION COMPLETE!')
                    self.get_logger().info(f'  Max error: {max_error:.4f} rad ({math.degrees(max_error):.2f} deg)')
                    self.get_logger().info(f'  Position: [{self.current_joint_pos[0]:.4f}, {self.current_joint_pos[1]:.4f}, {self.current_joint_pos[2]:.4f}]')
                    self.get_logger().info('='*60)
                    self.get_logger().info('Phase 2: Starting TORQUE TRAJECTORY execution...')
                    self.phase = 'PUBLISHING'
                    self.trajectory_start_time = time.time()
        else:
            self.stabilization_start_time = None
        
        # Log progress every 1 second
        if not hasattr(self, 'last_stab_log'):
            self.last_stab_log = time.time()
        if time.time() - self.last_stab_log >= 1.0:
            self.get_logger().info(
                f'STABILIZING... | Pos: [{self.current_joint_pos[0]:6.3f}, {self.current_joint_pos[1]:6.3f}, {self.current_joint_pos[2]:6.3f}] | '
                f'Err: {max_error:.4f} rad ({math.degrees(max_error):.2f} deg)'
            )
            self.last_stab_log = time.time()
    
    def final_stabilize_phase(self):
        """PID control to stabilize at final position after trajectory completion"""
        # Calculate position errors
        errors = [self.final_position[i] - self.current_joint_pos[i] for i in range(3)]
        
        # Update integral with anti-windup
        for i in range(3):
            self.integral_error[i] += errors[i] * self.dt
            self.integral_error[i] = max(-self.max_integral[i], 
                                        min(self.max_integral[i], self.integral_error[i]))
        
        # Calculate derivative
        derivative_error = [(errors[i] - self.previous_error[i]) / self.dt for i in range(3)]
        
        # PID control law
        torques = [
            self.kp[i] * errors[i] + 
            self.ki[i] * self.integral_error[i] + 
            self.kd[i] * derivative_error[i]
            for i in range(3)
        ]
        
        # Saturate torques
        torques = [max(-self.max_torques[i], min(self.max_torques[i], torques[i])) 
                   for i in range(3)]
        
        # Publish torques
        msg1 = Float64MultiArray()
        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()
        
        msg1.data = [torques[0]]
        msg2.data = [torques[1]]
        msg3.data = [torques[2]]
        
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        
        # Update previous error
        self.previous_error = errors.copy()
        
        # Check if stabilized
        max_error = max(abs(e) for e in errors)
        if max_error < self.position_threshold:
            if self.stabilization_start_time is None:
                self.stabilization_start_time = time.time()
            elif time.time() - self.stabilization_start_time >= self.min_stabilization_time:
                if not self.is_stabilized:
                    self.is_stabilized = True
                    self.get_logger().info('='*60)
                    self.get_logger().info(f'✓ FINAL STABILIZATION COMPLETE!')
                    self.get_logger().info(f'  Max error: {max_error:.4f} rad ({math.degrees(max_error):.2f} deg)')
                    self.get_logger().info(f'  Final position: [{self.current_joint_pos[0]:.4f}, {self.current_joint_pos[1]:.4f}, {self.current_joint_pos[2]:.4f}]')
                    self.get_logger().info('='*60)
                    self.get_logger().info('Arm is now stabilized at final position. Holding for 5 seconds...')
                    
                    self.finished = True
                    self.timer.cancel()
                    # Hold final position for 5 seconds, then shutdown
                    self.create_timer(5.0, self.shutdown)
        else:
            self.stabilization_start_time = None
        
        # Log progress every 1 second
        if not hasattr(self, 'last_final_stab_log'):
            self.last_final_stab_log = time.time()
        if time.time() - self.last_final_stab_log >= 1.0:
            self.get_logger().info(
                f'FINAL STABILIZING... | Pos: [{self.current_joint_pos[0]:6.3f}, {self.current_joint_pos[1]:6.3f}, {self.current_joint_pos[2]:6.3f}] | '
                f'Err: {max_error:.4f} rad ({math.degrees(max_error):.2f} deg)'
            )
            self.last_final_stab_log = time.time()
    
    def publish_phase(self):
        """Publish torque values from CSV trajectory"""
        # Calculate elapsed time since trajectory started
        elapsed = time.time() - self.trajectory_start_time
        
        # Find the closest time index
        while (self.current_row < len(self.torque_data) - 1 and 
               self.torque_data[self.current_row]['t'] < elapsed):
            self.current_row += 1
        
        # Get current row data
        current_data = self.torque_data[self.current_row]
        
        # Log actual position for this row
        log_entry = current_data.copy()
        log_entry['actual_p1'] = self.current_joint_pos[0]
        log_entry['actual_p2'] = self.current_joint_pos[1]
        log_entry['actual_p3'] = self.current_joint_pos[2]
        self.logged_rows.append(log_entry)
        
        # Create messages
        msg1 = Float64MultiArray()
        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()
        
        msg1.data = [current_data['tau1']]
        msg2.data = [current_data['tau2']]
        msg3.data = [current_data['tau3']]
        
        # Publish
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        
        # Log every 50 rows (every 0.5 seconds at 100 Hz)
        if self.current_row % 50 == 0:
            # Calculate tracking error
            pos_error = [
                current_data['dp1'] - self.current_joint_pos[0],
                current_data['dp2'] - self.current_joint_pos[1],
                current_data['dp3'] - self.current_joint_pos[2]
            ]
            max_error = max(abs(e) for e in pos_error)
            
            self.get_logger().info(
                f'Row {self.current_row}/{len(self.torque_data)} | t={current_data["t"]:.2f}s | '
                f'Torques: [{current_data["tau1"]:5.1f}, {current_data["tau2"]:5.1f}, {current_data["tau3"]:5.1f}] | '
                f'Tracking Err: {max_error:.4f} rad'
            )
        
        # Check if trajectory is complete
        if self.current_row >= len(self.torque_data) - 1:
            self.get_logger().info('='*60)
            self.get_logger().info(f'✓ Trajectory execution COMPLETE! Published {len(self.torque_data)} points')
            self.get_logger().info('='*60)
            
            # Save logged data to CSV
            self.save_to_csv()
            
            # Transition to final stabilization phase
            self.phase = 'FINAL_STABILIZING'
            # Use ACTUAL current position instead of desired position
            self.final_position = [
                self.current_joint_pos[0],
                self.current_joint_pos[1],
                self.current_joint_pos[2]
            ]
            self.get_logger().info('Phase 3: STABILIZING at actual current position...')
            self.get_logger().info(f'Target final position (actual): [{self.final_position[0]:.4f}, {self.final_position[1]:.4f}, {self.final_position[2]:.4f}] rad')
            self.get_logger().info(f'Desired last position was:      [{self.torque_data[-1]["dp1"]:.4f}, {self.torque_data[-1]["dp2"]:.4f}, {self.torque_data[-1]["dp3"]:.4f}] rad')
            
            # Reset stabilization variables
            self.integral_error = [0.0, 0.0, 0.0]
            self.previous_error = [0.0, 0.0, 0.0]
            self.is_stabilized = False
            self.stabilization_start_time = None
    
    def save_to_csv(self):
        """Save the logged data with actual positions to a new CSV file
        Output format (column-wise):
            dp1,dp2,dp3,tau1,tau2,tau3,actual_p1,actual_p2,actual_p3
            value1,value1,value1,value1,value1,value1,value1,value1,value1
            value2,value2,value2,value2,value2,value2,value2,value2,value2
            ...
        """
        try:
            if len(self.logged_rows) == 0:
                self.get_logger().warn('No data to save')
                return
            
            # Define column order (as requested: dp1, dp2, dp3, tau1, tau2, tau3, actual_p1, actual_p2, actual_p3)
            columns = ['dp1', 'dp2', 'dp3', 'tau1', 'tau2', 'tau3', 'actual_p1', 'actual_p2', 'actual_p3']
            
            # Write to CSV
            with open(self.output_csv_path, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=columns, extrasaction='ignore')
                writer.writeheader()
                writer.writerows(self.logged_rows)
            
            self.get_logger().info(f'✓ Saved trajectory data with actual positions to: {self.output_csv_path}')
            self.get_logger().info(f'  Total rows saved: {len(self.logged_rows)}')
            
            # Calculate and log statistics
            self._log_tracking_statistics()
            
        except Exception as e:
            self.get_logger().error(f'Failed to save CSV: {e}')
    
    def _log_tracking_statistics(self):
        """Calculate and log tracking error statistics"""
        if len(self.logged_rows) == 0:
            return
        
        errors_j1 = [abs(row['dp1'] - row['actual_p1']) for row in self.logged_rows]
        errors_j2 = [abs(row['dp2'] - row['actual_p2']) for row in self.logged_rows]
        errors_j3 = [abs(row['dp3'] - row['actual_p3']) for row in self.logged_rows]
        
        self.get_logger().info('Tracking Error Statistics:')
        self.get_logger().info(f'  Joint 1: Max={max(errors_j1):.4f} rad, Mean={sum(errors_j1)/len(errors_j1):.4f} rad')
        self.get_logger().info(f'  Joint 2: Max={max(errors_j2):.4f} rad, Mean={sum(errors_j2)/len(errors_j2):.4f} rad')
        self.get_logger().info(f'  Joint 3: Max={max(errors_j3):.4f} rad, Mean={sum(errors_j3)/len(errors_j3):.4f} rad')
    
    def shutdown(self):
        """Shutdown the node"""
        # Send zero torques
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0]
        self.pub1.publish(zero_msg)
        self.pub2.publish(zero_msg)
        self.pub3.publish(zero_msg)
        
        # Save data if not already saved and in publishing phase
        if self.phase == 'PUBLISHING' and len(self.logged_rows) > 0:
            if not self.finished:  # Data not saved yet
                self.get_logger().info('Saving data before shutdown...')
                self.save_to_csv()
        
        self.get_logger().info('Node shutting down')
        rclpy.shutdown()

def main(args=None):
    # Parse command line arguments
    csv_file = None
    if args is None:
        args = sys.argv[1:]
    
    # Check if CSV file path is provided as command line argument
    if len(args) > 0 and not args[0].startswith('--ros-args'):
        csv_file = args[0]
        # Remove the csv file from args so rclpy doesn't try to parse it
        args = args[1:]
    
    rclpy.init(args=args)
    node = CSVTorquePublisher(csv_file=csv_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()