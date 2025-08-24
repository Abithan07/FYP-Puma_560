# URDF Example


This repo contains an example of a URDF file and a launch script to run it. (ROS 2)
This is part of a tutorial on creating URDF files, available at the links below:

YouTube:
[![See video for example](https://img.youtube.com/vi/CwdbsvcpOHM/0.jpg)](https://youtu.be/CwdbsvcpOHM)

Blog Post:
[https://articulatedrobotics.xyz/ready-for-ros-7-urdf/](https://articulatedrobotics.xyz/ready-for-ros-7-urdf/)



## How To Run


1. `cd` into `<repo_directory>/urdf_example` Build the package with colcon. `colcon build`
2. Source the `setup.bash` file`source install/setup.bash` and Launch the `robot_state_publisher` launch file with `ros2 launch urdf_example rsp.launch.py`.
3. In a new terminal, launch `joint_state_publisher_gui` with `ros2 run joint_state_publisher_gui joint_state_publisher_gui`. You may need to install it if you don't have it already.
4. In a new terminal, launch RViz with `rviz2`

To replicate the RViz display shown in the video you will want to
- Set your fixed frame to `world`
- Add a `RobotModel` display, with the topic set to `/robot_description`, and alpha set to 0.8
- Add a `TF` display with names enabled.

`source /opt/ros/jazzy/setup.bash`
`source ~/ros2_ws/urdf_example/install/setup.bash`

spawn robot in gazebo
`ros2 run ros_gz_sim create -topic robot_description -name puma560`

installing gazebo harmonic and dependancies for ros2 jazzy
```bash
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-twist-mux
sudo apt install ros-jazzy-twist-stamper 
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
```
