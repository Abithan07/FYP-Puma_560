import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import os
import sys

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
        
        # Load torque data from CSV
        self.torque_data = []
        self.load_csv_data(csv_file)
        
        # Current row index
        self.current_row = 0
        self.finished = False
        
        # Timer to publish every 0.01 seconds (100 Hz)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.publish_torques)
        
        self.get_logger().info(f'CSV Torque Publisher started. Loaded {len(self.torque_data)} rows from {csv_file}')
        self.get_logger().info(f'Publishing at {1/self.timer_period} Hz')
    
    def load_csv_data(self, csv_file):
        """Load torque values from CSV file"""
        try:
            # Check if file exists
            if not os.path.exists(csv_file):
                self.get_logger().error(f'CSV file not found: {csv_file}')
                return
            
            with open(csv_file, 'r') as file:
                csv_reader = csv.DictReader(file)
                
                # Read all rows
                for row in csv_reader:
                    try:
                        torques = {
                            'joint_1': float(row['tau1']),
                            'joint_2': float(row['tau2']),
                            'joint_3': float(row['tau3'])
                        }
                        self.torque_data.append(torques)
                    except (ValueError, KeyError) as e:
                        self.get_logger().warn(f'Skipping invalid row: {e}')
                        continue
            
            if len(self.torque_data) == 0:
                self.get_logger().error('No valid data found in CSV file')
        
        except Exception as e:
            self.get_logger().error(f'Error loading CSV file: {e}')
    
    def publish_torques(self):
        """Publish torque values from current row"""
        if len(self.torque_data) == 0:
            self.get_logger().warn('No torque data available')
            self.timer.cancel()
            return
        
        if self.finished:
            return
        
        # Get current row data
        current_torques = self.torque_data[self.current_row]
        
        # Create messages
        msg1 = Float64MultiArray()
        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()
        
        msg1.data = [current_torques['joint_1']]
        msg2.data = [current_torques['joint_2']]
        msg3.data = [current_torques['joint_3']]
        
        # Publish
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        
        # Log every 100 rows (every 1 second at 100 Hz)
        if self.current_row % 100 == 0:
            self.get_logger().info(
                f'Row {self.current_row}/{len(self.torque_data)}: '
                f'J1={current_torques["joint_1"]:.2f}, '
                f'J2={current_torques["joint_2"]:.2f}, '
                f'J3={current_torques["joint_3"]:.2f}'
            )
        
        # Move to next row
        self.current_row += 1
        
        # Stop when reaching end of data
        if self.current_row >= len(self.torque_data):
            self.get_logger().info(f'Finished publishing all {len(self.torque_data)} rows')
            self.get_logger().info('Publishing complete. Shutting down...')
            self.finished = True
            self.timer.cancel()
            # Shutdown after a brief delay to ensure last messages are sent
            self.create_timer(0.1, self.shutdown)
    
    def shutdown(self):
        """Shutdown the node"""
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