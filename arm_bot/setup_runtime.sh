#!/usr/bin/env bash
# Project-only compatibility overrides for the installed ROS/inference stack.
set -eo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname -- "$WS")"
source "$WS/scripts/desktop_environment.sh"
BASE_PYTHON="${PUMA_BASE_PYTHON:-/usr/bin/python3}"
"$BASE_PYTHON" -m venv --system-site-packages "$PROJECT_ROOT/.venv-ros"
"$PROJECT_ROOT/.venv-ros/bin/python3" -m pip install --no-cache-dir \
    --disable-pip-version-check -r "$PROJECT_ROOT/requirements/ros-overlay.txt"
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
"$PROJECT_ROOT/.venv-ros/bin/python3" "$PROJECT_ROOT/tools/check_runtime.py"
