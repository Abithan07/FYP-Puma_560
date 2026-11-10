#!/usr/bin/env bash
# Legacy data-collection workflow. See docs/WORKFLOWS.md for the final demo.
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# != 1 || ! "$1" =~ ^[0-9]{1,6}$ ]]; then
    echo 'Usage: bash arm_bot/pipeline.sh <path_number>' >&2
    exit 2
fi
PATH_ID="$((10#$1))"
LAUNCH_PID=''

stop_simulation() {
    if [[ -n "$LAUNCH_PID" ]]; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
        for ((attempt = 0; attempt < 5; attempt++)); do
            kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
            sleep 1
        done
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
        sleep 2
        kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
        LAUNCH_PID=''
    fi
}
trap stop_simulation EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cd -- "$WS"
colcon build --symlink-install --packages-select arm_bot
source install/setup.bash
setsid ros2 launch arm_bot robot_gui.launch.py &
LAUNCH_PID=$!

echo 'Waiting for controllers to become ACTIVE...'
ready=false
for ((attempt = 0; attempt < 40; attempt++)); do
    controllers="$(timeout 5s ros2 control list_controllers 2>/dev/null | sed -r 's/\x1B\[[0-9;]*[mK]//g' || true)"
    if [[ "$(printf '%s\n' "$controllers" | grep -Ec 'joint_[123]_controller.*[[:space:]]active' || true)" == 3 ]]; then
        ready=true
        break
    fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || { echo 'ERROR: Simulation exited.' >&2; exit 1; }
    sleep 0.5
done
"$ready" || { echo 'ERROR: Controllers did not become active.' >&2; exit 1; }

bash src/scripts/dataset_generator.sh "$PATH_ID"
LOG_FILE="$(python3 - "$WS/src/scripts/logs" "$PATH_ID" <<'PY'
from pathlib import Path
import re
import sys
files = [p for p in Path(sys.argv[1]).glob('path_*_log_*.csv')
         if (m := re.fullmatch(r'path_(\d+)_log_\d+\.csv', p.name))
         and int(m[1]) == int(sys.argv[2])]
if files:
    print(max(files, key=lambda p: p.stat().st_mtime))
PY
)"
[[ -n "$LOG_FILE" ]] || { echo 'ERROR: No log file found.' >&2; exit 1; }

echo "Waiting for log file to stabilize: $LOG_FILE"
previous_size=0
stable_count=0
for ((attempt = 0; attempt < 120; attempt++)); do
    current_size="$(stat -c%s "$LOG_FILE")"
    if [[ "$current_size" == "$previous_size" ]]; then
        stable_count=$((stable_count + 1))
    else
        stable_count=0
    fi
    (( stable_count >= 3 )) && break
    previous_size="$current_size"
    sleep 1
done
(( stable_count >= 3 )) || { echo 'ERROR: Log file did not finish writing.' >&2; exit 1; }
stop_simulation
python3 src/scripts/process_dataset.py "$PATH_ID"
echo 'PIPELINE COMPLETED SUCCESSFULLY'
