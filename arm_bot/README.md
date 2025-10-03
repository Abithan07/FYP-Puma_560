## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).



ros2 run joint_state_publisher_gui joint_state_publisher_gui
# In another terminal
ros2 launch your_package robot.launch.py
# In another terminal  
ros2 run rviz2 rviz2


# Using the new launch file
ros2 launch your_package_name robot_gui.launch.py


# To control your current effort controllers:
ros2 topic pub /joint_1_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"


colcon build
source install/setup.bash
ros2 launch arm_bot robot_gui.launch.py
python3 /home/abithan_ubuntu/ros2_ws2/arm_bot/scripts/torque_publisher.py