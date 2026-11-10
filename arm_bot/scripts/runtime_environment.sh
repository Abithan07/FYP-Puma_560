#!/usr/bin/env bash
# Keep all Python children on the same project runtime, including ROS executables.
_puma_select_python_runtime() {
    local project_root python_path
    project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
    if [[ -n "${PUMA_PYTHON:-}" ]]; then
        python_path="$(command -v -- "$PUMA_PYTHON")" || {
            echo "ERROR: Python interpreter not found: $PUMA_PYTHON" >&2
            return 1
        }
    elif [[ -x "$project_root/.venv-ros/bin/python3" ]]; then
        python_path="$project_root/.venv-ros/bin/python3"
    else
        python_path="$(command -v python3)" || return 1
    fi
    # Preserve the venv executable path: resolving its symlink loses venv selection.
    export PUMA_PYTHON="$python_path"
    export PATH="$(dirname -- "$python_path"):$PATH"
}

_puma_select_python_runtime
unset -f _puma_select_python_runtime
