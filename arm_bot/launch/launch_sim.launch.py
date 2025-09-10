import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node



def generate_launch_description():


    package_name='arm_bot'

    # Robot State Publisher for simulation
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Remove joystick and twist_mux - not needed for arms

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_arm'],
        output='screen'
    )

    # Spawn arm controllers
    joint_1_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_1_controller"],
    )

    joint_2_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["joint_2_controller"],
    )

    joint_3_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_3_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Delay controller spawning until robot is spawned
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_1_spawner, joint_2_spawner, joint_3_spawner, joint_broad_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        delayed_spawners
    ])
