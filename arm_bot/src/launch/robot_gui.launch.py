from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    pkg_name = 'arm_bot'  # Replace with your actual package name
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to XACRO file
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    
    controller_yaml = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    sim_yaml = os.path.join(pkg_share, 'config', 'gaz_ros2_ctl_use_sim.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'main.rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            # Optionally, you can specify a world file here
            # 'world': os.path.join(pkg_share, 'worlds', 'your_world.world')
        }.items()
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arm_bot'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',  # Enable ROS2 control for torque input
            description='Use ROS2 control'
        ),

        DeclareLaunchArgument(
            'sim_mode',
            default_value='true',
            description='Enable simulation mode'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', xacro_file,
                    ' use_ros2_control:=', LaunchConfiguration('use_ros2_control'),
                    ' sim_mode:=', LaunchConfiguration('sim_mode')
                ])
            }]
        ),

        # Launch Gazebo first
        gazebo_launch,

        # Wait for Gazebo to start, then spawn the robot
        TimerAction(
            period=5.0,
            actions=[spawn_entity]
        ),

        # Wait for gazebo_ros2_control plugin to load, then spawn controllers
        TimerAction(
            period=8.0,  # Give Gazebo + plugin time to fully initialize
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=10.0,  # Spawn joint controllers after state broadcaster
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_1_controller'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_2_controller'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_3_controller'],
                    output='screen',
                ),
            ]
        ),

        # RViz2 with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])