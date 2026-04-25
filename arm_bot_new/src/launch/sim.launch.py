"""
sim.launch.py — Launches the ideal PUMA 560 Gazebo simulation.

What it starts:
  1. Gazebo (Ignition Fortress) with puma_world.sdf
  2. Robot State Publisher (publishes TF from URDF)
  3. Clock bridge (gz → ROS2 /clock)
  4. Spawns the robot URDF in Gazebo
  5. joint_state_broadcaster  (publishes /joint_states)
  6. joint_{1,2,3}_controller (ForwardCommandController, effort interface)

Run with:
  ros2 launch arm_bot_new sim.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name  = 'arm_bot_new'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file      = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    controller_yaml = os.path.join(pkg_share, 'config',      'my_controllers.yaml')
    world_file      = os.path.join(pkg_share, 'worlds',      'puma_world.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Make Gazebo find package meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'meshes'), ':', os.path.join(pkg_share, '..')],
    )

    # Parse URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file,
        ' use_ros2_control:=true',
        ' sim_mode:=true',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ── Nodes ──────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': '-r -v 1 ' + world_file}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'arm_bot_new',
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # Bridge /clock so ros2_control uses sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Controller spawners (order matters: broadcaster first, then joints)
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    j1_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_1_controller', '--param-file', controller_yaml],
        output='screen',
    )
    j2_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_2_controller', '--param-file', controller_yaml],
        output='screen',
    )
    j3_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_3_controller', '--param-file', controller_yaml],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation time'),
        gz_resource_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        # Spawn broadcaster once robot is in Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[jsb_spawner],
            )
        ),
        # Spawn joint controllers once broadcaster is active
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[j1_spawner, j2_spawner, j3_spawner],
            )
        ),
    ])
