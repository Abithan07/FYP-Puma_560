cmake_minimum_required(VERSION 3.5)
project(arm_bot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config description launch worlds meshes scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/controllers/torque_publisher_ctc.py
    scripts/controllers/torque_publisher_dnn.py
    scripts/controllers/pid_controller_trajectory.py
    scripts/analysis/analyze_dnn_performance.py
    scripts/analysis/analyze_ctc_performance.py
    scripts/shared/continuous_logger_triggered.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/shared/project_paths.py scripts/shared/inverse_dynamics_model.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/shared/line_drawer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_drawer
)

ament_package()
