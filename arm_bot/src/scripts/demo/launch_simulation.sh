#!/usr/bin/env bash
set -eo pipefail

DEMO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$DEMO_DIR/../../.." && pwd)"
source "$WS/scripts/desktop_environment.sh"
if [[ ! -f "$WS/install/setup.bash" ]]; then
    echo "Build arm_bot first: cd '$WS' && colcon build --symlink-install" >&2
    exit 1
fi
source "$WS/install/setup.bash"
source "$WS/scripts/runtime_environment.sh"
exec ros2 launch arm_bot gazebo.launch.py
