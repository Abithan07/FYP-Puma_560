#!/usr/bin/env bash
set -eo pipefail

WS="$HOME/Desktop/FYP-Puma_560/arm_bot"
IDS=('641' '647' '648' '649' '650' '652' '653' '659' '660' '661' '663' '665' '666' '677' '682' '687' '697' '698' '699' '701' '704' '705' '711' '712' '715' '720' '724' '725' '727' '728' '733' '741' '742' '744' '750' '758' '759' '760' '761' '762' '766' '770' '773' '780' '782' '783' '786' '788' '799' '800')

LOG_DIR="$WS/run_logs"
STATUS_FILE="/tmp/arm_bot_controller_status.txt"

mkdir -p "$LOG_DIR"

open_gazebo() {
    gnome-terminal --title="Gazebo + RViz" -- bash -lc "
        cd '$WS'
        colcon build --symlink-install &&
        source install/setup.bash &&
        ros2 launch arm_bot gazebo.launch.py
    "

    echo "Waiting 10 seconds for Gazebo startup..."
    sleep 10
}

reset_gazebo() {
    echo "Resetting Gazebo simulation..."

    source "$WS/install/setup.bash"

    ros2 service call /world/default/control gz_msgs/srv/WorldControl \
    "{world_control: {reset: {all: true}}}" >/dev/null 2>&1 || true

    sleep 3
}

clear_rviz_markers() {
    echo "Clearing RViz drawings..."

    source "$WS/install/setup.bash"

    # Publish empty marker array to clear all visualizations
    ros2 topic pub -1 /visualization_marker_array visualization_msgs/MarkerArray \
    "{markers: []}" >/dev/null 2>&1 || true

    sleep 1
}

stop_gazebo() {
    echo "Stopping Gazebo/RViz..."

    pkill -INT -f 'ros2 launch arm_bot gazebo.launch.py' 2>/dev/null || true
    sleep 2

    pkill -TERM -f 'ros2 launch arm_bot gazebo.launch.py' 2>/dev/null || true
    pkill -TERM -f 'ign gazebo' 2>/dev/null || true
    pkill -TERM -f 'rviz2' 2>/dev/null || true
    pkill -TERM -f 'robot_state_publisher' 2>/dev/null || true

    sleep 5
}

run_controller() {
    local name="$1"
    local cmd="$2"
    local id="$3"

    rm -f "$STATUS_FILE"

    echo "Running $name for path_$id..."

    gnome-terminal --title="$name path_$id" -- bash -lc "
        cd '$WS'
        source install/setup.bash

        $cmd 2>&1 | tee '$LOG_DIR/${name}_path_${id}.log'

        rc=\${PIPESTATUS[0]}
        echo \$rc > '$STATUS_FILE'

        echo
        echo '$name completed.'
    "

    while [[ ! -f "$STATUS_FILE" ]]; do
        sleep 1
    done

    RC=$(cat "$STATUS_FILE")

    if [[ "$RC" != "0" ]]; then
        echo "ERROR: $name failed for path_$id"
        echo "Check log: $LOG_DIR/${name}_path_${id}.log"
        exit 1
    fi

    echo "$name finished for path_$id."
}

cleanup() {
    stop_gazebo
}
trap cleanup EXIT

# Start Gazebo once at the beginning
open_gazebo

# Wait for user to confirm Gazebo is ready
echo "Gazebo started. Press Enter when ready to start controllers..."
read -r

# Now run all controllers in sequence
for id in "${IDS[@]}"; do
    CSV="$WS/src/scripts/Joint_states/J/path_${id}_joint_states.csv"

    echo "======================================"
    echo "Starting trajectory path_$id"
    echo "CSV: $CSV"
    echo "======================================"

    if [[ ! -f "$CSV" ]]; then
        echo "ERROR: CSV not found:"
        echo "$CSV"
        exit 1
    fi

    run_controller \
        "dnn" \
        "ros2 run arm_bot torque_publisher_dnn.py --csv-path '$CSV'" \
        "$id"

    clear_rviz_markers
    reset_gazebo

    run_controller \
        "ctc" \
        "ros2 run arm_bot torque_publisher_ctc.py --csv-path '$CSV'" \
        "$id"

    clear_rviz_markers
    reset_gazebo

    run_controller \
        "pid" \
        "python3 src/scripts/controller/pid_controller/pid_controller_trajectory.py --trajectory-csv '$CSV'" \
        "$id"

    clear_rviz_markers
    reset_gazebo

    echo "Completed all controllers for path_$id"
done

echo "======================================"
echo "ALL TRAJECTORIES COMPLETED"
echo "======================================"