# #!/bin/bash

# set -e

# # Check if argument is given
# if [ -z "$1" ]; then
#   echo "Usage: ./pipeline.sh <number>"
#   exit 1
# fi

# DATASET_ID=$1

# echo "Building workspace..."
# colcon build

# echo "Sourcing workspace..."
# source install/setup.bash

# echo "Launching robot..."
# ros2 launch arm_bot robot_gui.launch.py &
# LAUNCH_PID=$!

# echo "Waiting for system to be ready..."

# # Better than sleep (wait for ROS topic)
# until ros2 topic list | grep -q "/joint_states"
# do
#   sleep 1
# done

# echo "Running dataset generator with ID: $DATASET_ID"
# ./src/scripts/dataset_generator.sh $DATASET_ID

# wait $LAUNCH_PID

# #!/bin/bash

# set -e

# # ================================
# # Input check
# # ================================
# if [ -z "$1" ]; then
#   echo "Usage: ./pipeline.sh <path_number>"
#   exit 1
# fi

# PATH_ID=$1

# echo "======================================"
# echo "RUNNING PIPELINE FOR PATH: $PATH_ID"
# echo "======================================"

# # ================================
# # Build & Source
# # ================================
# colcon build
# source install/setup.bash

# # ================================
# # Launch ROS2 (background)
# # ================================
# setsid ros2 launch arm_bot robot_gui.launch.py &
# LAUNCH_PID=$!

# # ================================
# # Wait for system ready
# # ================================
# echo "Waiting for ROS system..."
# until ros2 topic list | grep -q "/joint_states"
# do
#   sleep 1
# done

# echo "System ready."

# # ================================
# # Run dataset generator
# # ================================
# ./src/scripts/dataset_generator.sh $PATH_ID

# echo "Dataset generation completed."

# # ================================
# # Find latest log file
# # ================================
# LOG_FILE=$(ls -t ~/Desktop/arm_bot/src/scripts/logs/path_${PATH_ID}_log_*.csv | head -n 1)

# if [ -z "$LOG_FILE" ]; then
#   echo "❌ No log file found!"
#   kill $LAUNCH_PID
#   exit 1
# fi

# echo "Monitoring log file: $LOG_FILE"

# # ================================
# # Wait for file write completion
# # ================================
# echo "Waiting for log file to stabilize..."

# PREV_SIZE=0
# while true; do
#   CUR_SIZE=$(stat -c%s "$LOG_FILE")

#   if [ "$CUR_SIZE" -eq "$PREV_SIZE" ]; then
#     break
#   fi

#   PREV_SIZE=$CUR_SIZE
#   sleep 1
# done

# echo "✅ Log file write completed."

# # ================================
# # Kill ROS2 / Gazebo
# # ================================
# echo "Stopping ROS2 + Gazebo..."

# kill -- -$LAUNCH_PID

# sleep 2

# # Fallback cleanup (just in case)
# pkill -f ign || true
# pkill -f gazebo || true

# echo "✅ ROS2 & Gazebo stopped."

# # ================================
# # Run Python post-processing
# # ================================
# echo "Starting post-processing..."

# python3 src/scripts/process_dataset.py $PATH_ID

# echo "======================================"
# echo "PIPELINE COMPLETED SUCCESSFULLY"
# echo "======================================"


#!/bin/bash

set -e

# ================================
# Cleanup handler (VERY IMPORTANT)
# ================================
cleanup() {
  echo "Cleaning up processes..."

  # Kill ROS launch group
  if [ ! -z "$LAUNCH_PID" ]; then
    kill -- -$LAUNCH_PID 2>/dev/null || true
  fi

  # Kill remaining processes
  pkill -f "ros2" 2>/dev/null || true
  pkill -f ign 2>/dev/null || true
  pkill -f gazebo 2>/dev/null || true
}

trap cleanup EXIT

# ================================
# Input check
# ================================
if [ -z "$1" ]; then
  echo "Usage: ./pipeline.sh <path_number>"
  exit 1
fi

PATH_ID=$1

echo "======================================"
echo "RUNNING PIPELINE FOR PATH: $PATH_ID"
echo "======================================"

# ================================
# Build & Source
# ================================
colcon build
source install/setup.bash

# ================================
# Launch ROS2 (new process group)
# ================================
setsid ros2 launch arm_bot robot_gui.launch.py &
LAUNCH_PID=$!

# ================================
# Wait for system ready (SAFE)
# ================================
# echo "Waiting for ROS system..."

# for i in {1..30}; do
#   if ros2 topic list 2>/dev/null | grep -q "/joint_states"; then
#     echo "System ready."
#     break
#   fi
#   sleep 1
# done

# echo "Waiting for controller manager..."

# for i in {1..30}; do
#   if ros2 service list 2>/dev/null | grep -q "/controller_manager/list_controllers"; then
#     echo "Controller manager ready."
#     break
#   fi
#   sleep 0.5
# done

echo "Waiting for controllers to become ACTIVE..."

for i in {1..40}; do
  if ros2 control list_controllers 2>/dev/null | grep -q "active"; then
    echo "Controllers are ACTIVE."
    break
  fi
  sleep 0.5
done

# ================================
# Run dataset generator
# ================================
./src/scripts/dataset_generator.sh $PATH_ID

echo "Dataset generation completed."

# ================================
# Find latest log file
# ================================
LOG_FILE=$(ls -t ~/Desktop/arm_bot/src/scripts/logs/path_${PATH_ID}_log_*.csv 2>/dev/null | head -n 1)

if [ -z "$LOG_FILE" ]; then
  echo "❌ No log file found!"
  exit 1
fi

echo "Monitoring log file: $LOG_FILE"

# ================================
# Wait for file write completion (ROBUST)
# ================================
echo "Waiting for log file to stabilize..."

PREV_SIZE=0
STABLE_COUNT=0

while true; do
  CUR_SIZE=$(stat -c%s "$LOG_FILE")

  if [ "$CUR_SIZE" -eq "$PREV_SIZE" ]; then
    STABLE_COUNT=$((STABLE_COUNT+1))
  else
    STABLE_COUNT=0
  fi

  if [ "$STABLE_COUNT" -ge 3 ]; then
    break
  fi

  PREV_SIZE=$CUR_SIZE
  sleep 1
done

echo "✅ Log file write completed."

# ================================
# Stop ROS2 + Gazebo cleanly
# ================================
echo "Stopping ROS2 + Gazebo..."

# Stop ros2 CLI calls first (prevents your error spam)
pkill -f "ros2 topic" 2>/dev/null || true

# Kill entire process group
kill -- -$LAUNCH_PID 2>/dev/null || true

sleep 3

# Extra cleanup
pkill -f ign 2>/dev/null || true
pkill -f gazebo 2>/dev/null || true

echo "✅ ROS2 & Gazebo stopped."

# ================================
# Run Python post-processing
# ================================
echo "Starting post-processing..."

python3 src/scripts/process_dataset.py $PATH_ID

echo "======================================"
echo "PIPELINE COMPLETED SUCCESSFULLY"
echo "======================================"