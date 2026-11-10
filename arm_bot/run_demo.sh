#!/usr/bin/env bash
# Final presentation launcher. Build the main workspace and start the GUI.
set -eo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/scripts/desktop_environment.sh"
case "${1:-}" in
    --help|-h)
        echo 'Usage: bash arm_bot/run_demo.sh [--no-build]'
        echo 'Source ROS Humble first. The default builds arm_bot and starts the final GUI.'
        exit 0 ;;
    --no-build|'') ;;
    *) echo 'Usage: bash arm_bot/run_demo.sh [--no-build]' >&2; exit 2 ;;
esac
(( $# <= 1 )) || { echo 'ERROR: Too many arguments.' >&2; exit 2; }
for command in colcon ros2 python3; do
    command -v "$command" >/dev/null || { echo "ERROR: Missing $command. Source ROS Humble first." >&2; exit 1; }
done
# Resolve relative overrides before moving into the workspace.
for variable in PUMA_PROJECT_ROOT PUMA_DATA_DIR PUMA_MODEL_DIR PUMA_LOG_DIR; do
    if [[ -n "${!variable:-}" ]]; then
        printf -v "$variable" '%s' "$(realpath -m -- "${!variable}")"
        export "$variable"
    fi
done
cd -- "$WS"
if [[ "${1:-}" != --no-build ]]; then
    colcon build --symlink-install --packages-select arm_bot
fi
[[ -f install/setup.bash ]] || { echo 'ERROR: Build arm_bot before using --no-build.' >&2; exit 1; }
source install/setup.bash
exec python3 src/scripts/demo/traj_gui.py
