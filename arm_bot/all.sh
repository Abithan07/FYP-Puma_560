#!/usr/bin/env bash
# Compare DNN, CTC, and PID controllers on the same trajectory inputs.
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/scripts/desktop_environment.sh"
PROJECT_ROOT="$(realpath -m -- "${PUMA_PROJECT_ROOT:-$(dirname -- "$WS")}")"
DATA_DIR="$(realpath -m -- "${PUMA_DATA_DIR:-$PROJECT_ROOT/data}")"
export PUMA_DATA_DIR="$DATA_DIR"
CSV_DIR="$DATA_DIR/trajectories/Joint_states/J"
LOG_DIR="$DATA_DIR/results/console"
JOB_SCRIPT="$WS/scripts/terminal_job.sh"
CONTROLLER_TIMEOUT="${PUMA_CONTROLLER_TIMEOUT:-900}"
IDS=('641' '647' '648' '649' '650' '652' '653' '659' '660' '661' '663' '665' '666' '677' '682' '687' '697' '698' '699' '701' '704' '705' '711' '712' '715' '720' '724' '725' '727' '728' '733' '741' '742' '744' '750' '758' '759' '760' '761' '762' '766' '770' '773' '780' '782' '783' '786' '788' '799' '800')
CHECK_ONLY=false
REQUESTED_IDS=()

usage() {
    cat <<'HELP'
Usage: bash arm_bot/all.sh [--check] [--csv-dir DIR] [PATH_ID ...]

Without path IDs, use the original 50-path comparison batch.
--csv-dir DIR  Folder containing path_NNN_joint_states.csv inputs.
--check        Check inputs, models, and commands without building or launching.

Source your ROS setup before running. Requires a desktop and gnome-terminal.
PUMA_CONTROLLER_TIMEOUT sets the per-controller timeout in seconds (default 900).
HELP
}

while (( $# )); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --check) CHECK_ONLY=true; shift ;;
        --csv-dir)
            [[ $# -ge 2 ]] || { echo 'ERROR: --csv-dir requires a directory.' >&2; exit 2; }
            CSV_DIR="$2"; shift 2 ;;
        *)
            [[ "$1" =~ ^[0-9]{1,6}$ ]] || { echo "ERROR: Invalid path ID: $1" >&2; exit 2; }
            printf -v id '%03d' "$((10#$1))"
            REQUESTED_IDS+=("$id"); shift ;;
    esac
done
if (( ${#REQUESTED_IDS[@]} )); then
    IDS=("${REQUESTED_IDS[@]}")
fi
[[ "$CONTROLLER_TIMEOUT" =~ ^[1-9][0-9]*$ ]] || {
    echo 'ERROR: PUMA_CONTROLLER_TIMEOUT must be a positive integer.' >&2; exit 2;
}

# Resolve before changing to the workspace so relative --csv-dir paths work.
[[ -d "$CSV_DIR" ]] || { echo "ERROR: CSV directory not found: $CSV_DIR" >&2; exit 1; }
CSV_DIR="$(cd -- "$CSV_DIR" && pwd)"
if [[ -n "${PUMA_PROJECT_ROOT:-}" ]]; then
    PUMA_PROJECT_ROOT="$(cd -- "$PUMA_PROJECT_ROOT" && pwd)"
    export PUMA_PROJECT_ROOT
fi
for id in "${IDS[@]}"; do
    csv="$CSV_DIR/path_${id}_joint_states.csv"
    [[ -f "$csv" ]] || { echo "ERROR: CSV not found: $csv" >&2; exit 1; }
done
MODEL_DIR="${PUMA_MODEL_DIR:-${PUMA_PROJECT_ROOT:-$(dirname -- "$WS")}/models}"
[[ -d "$MODEL_DIR" ]] || { echo "ERROR: Model directory not found: $MODEL_DIR" >&2; exit 1; }
PUMA_MODEL_DIR="$(cd -- "$MODEL_DIR" && pwd)"
export PUMA_MODEL_DIR
MODEL_DIR="$PUMA_MODEL_DIR"
for model in fyp_jax_delan_50.jax best_GRUResidual.pt feature_scaler.pkl; do
    [[ -f "$MODEL_DIR/$model" ]] || { echo "ERROR: Model not found: $MODEL_DIR/$model" >&2; exit 1; }
done
for command in colcon ros2 gnome-terminal setsid; do
    command -v "$command" >/dev/null || { echo "ERROR: Missing command: $command. Source ROS first." >&2; exit 1; }
done
if "$CHECK_ONLY"; then
    echo "Preflight passed for ${#IDS[@]} trajectories in $CSV_DIR."
    exit 0
fi

cd -- "$WS"
colcon build --symlink-install --packages-select arm_bot
source "$WS/install/setup.bash"
mkdir -p "$LOG_DIR"
RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/arm_bot_compare.XXXXXXXX")"

stop_job() {
    local pid_file="$1" pid
    [[ -f "$pid_file" ]] || return 0
    read -r pid < "$pid_file"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    kill -INT -- "-$pid" 2>/dev/null || true
    for ((attempt = 0; attempt < 5; attempt++)); do
        kill -0 -- "-$pid" 2>/dev/null || return 0
        sleep 1
    done
    kill -TERM -- "-$pid" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
    local pid_file
    for pid_file in "$RUN_DIR"/*.pid; do
        stop_job "$pid_file"
    done
    rm -rf -- "$RUN_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_job() {
    local key="$1" title="$2" log="$3"
    shift 3
    gnome-terminal --title="$title" -- bash "$JOB_SCRIPT" \
        "$WS" "$RUN_DIR/$key.status" "$RUN_DIR/$key.pid" "$log" "$@"
}

reset_gazebo() {
    echo 'Resetting Gazebo simulation...'
    ros2 service call /world/default/control gz_msgs/srv/WorldControl \
        '{world_control: {reset: {all: true}}}' >/dev/null 2>&1 || true
    sleep 3
}

clear_rviz_markers() {
    ros2 topic pub -1 /visualization_marker_array visualization_msgs/MarkerArray \
        '{markers: []}' >/dev/null 2>&1 || true
    sleep 1
}

run_controller() {
    local name="$1" id="$2" key rc start
    shift 2
    key="${name}_path_${id}"
    echo "Running $name for path_$id..."
    start_job "$key" "$name path_$id" "$LOG_DIR/$key.log" "$@"
    start=$SECONDS
    while [[ ! -f "$RUN_DIR/$key.status" ]]; do
        if (( SECONDS - start >= CONTROLLER_TIMEOUT )); then
            echo "ERROR: $key timed out. Check $LOG_DIR/$key.log" >&2
            return 1
        fi
        sleep 1
    done
    read -r rc < "$RUN_DIR/$key.status"
    if [[ "$rc" != 0 ]]; then
        echo "ERROR: $key failed (exit $rc). Check $LOG_DIR/$key.log" >&2
        return 1
    fi
    rm -f -- "$RUN_DIR/$key.pid"
    echo "$name finished for path_$id."
}

start_job gazebo 'Gazebo + RViz' "$LOG_DIR/gazebo_current.log" \
    ros2 launch arm_bot gazebo.launch.py
echo 'Waiting 10 seconds for Gazebo startup...'
sleep 10
if [[ -f "$RUN_DIR/gazebo.status" ]]; then
    echo "ERROR: Gazebo exited. Check $LOG_DIR/gazebo_current.log" >&2
    exit 1
fi
echo 'Press Enter when Gazebo and all three controllers are ready...'
read -r

for id in "${IDS[@]}"; do
    CSV="$CSV_DIR/path_${id}_joint_states.csv"
    echo "Starting trajectory path_$id: $CSV"
    run_controller dnn "$id" ros2 run arm_bot torque_publisher_dnn.py --csv-path "$CSV"
    clear_rviz_markers
    reset_gazebo
    run_controller ctc "$id" ros2 run arm_bot torque_publisher_ctc.py --csv-path "$CSV"
    clear_rviz_markers
    reset_gazebo
    run_controller pid "$id" python3 src/scripts/controllers/pid_controller_trajectory.py --trajectory-csv "$CSV"
    clear_rviz_markers
    reset_gazebo
    echo "Completed all controllers for path_$id"
done
echo 'ALL TRAJECTORIES COMPLETED'
