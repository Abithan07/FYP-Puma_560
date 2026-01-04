from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    pkg_name = 'arm_bot'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to XACRO file
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    
    controller_yaml = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'main.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'empty_fortress.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set Gazebo resource path for meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'meshes'), ':', os.path.join(pkg_share, '..')]
    )

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file,
        ' use_ros2_control:=true',
        ' sim_mode:=true'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Gazebo Sim (Fortress) launch using ros_gz_sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r -v 1 ' + world_file,
        }.items()
    )

    # Node to spawn the robot in Gazebo Sim
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'arm_bot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )
    
    # Bridge for clock synchronization (required for ros2_control in simulation)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joint_1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_1_controller', '--param-file', controller_yaml],
        output='screen',
    )

    joint_2_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_2_controller', '--param-file', controller_yaml],
        output='screen',
    )

    joint_3_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_3_controller', '--param-file', controller_yaml],
        output='screen',
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Set environment variables
        gz_resource_path,

        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Launch Gazebo Sim
        gazebo_launch,
        
        # Bridge for clock
        bridge,

        # Robot State Publisher
        node_robot_state_publisher,

        # Spawn robot entity
        gz_spawn_entity,

        # Spawn joint_state_broadcaster after robot spawns
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        # Spawn controllers after joint_state_broadcaster is active
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    joint_1_controller_spawner,
                    joint_2_controller_spawner,
                    joint_3_controller_spawner,
                ],
            )
        ),

        # RViz2
        rviz_node,
    ])