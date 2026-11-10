#!/usr/bin/env bash
# Internal helper for all.sh: track one process group and report its exit status.
set -eo pipefail

if (( $# < 5 )); then
    echo 'Usage: terminal_job.sh WORKSPACE STATUS_FILE PID_FILE LOG_FILE COMMAND [ARGS...]' >&2
    exit 2
fi
workspace="$1"
status_file="$2"
pid_file="$3"
log_file="$4"
shift 4
job_pid=''

finish() {
    local rc=$?
    trap - EXIT
    if [[ -n "$job_pid" ]]; then
        kill -TERM -- "-$job_pid" 2>/dev/null || true
    fi
    # Atomic rename prevents the parent reading a partially written status.
    printf '%s\n' "$rc" > "$status_file.tmp"
    mv -- "$status_file.tmp" "$status_file"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cd -- "$workspace"
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/desktop_environment.sh"
source install/setup.bash
setsid "$@" > >(tee "$log_file") 2>&1 &
job_pid=$!
printf '%s\n' "$job_pid" > "$pid_file"
wait "$job_pid"
