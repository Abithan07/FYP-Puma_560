#!/usr/bin/env bash
# Run one legacy feedforward trajectory, an inclusive range, or a suffixed range.
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if (( $# < 1 || $# > 3 )); then
    echo 'Usage: dataset_generator.sh START [END [SUFFIX]]' >&2
    exit 2
fi
[[ "$1" =~ ^[0-9]{1,6}$ && "${2:-$1}" =~ ^[0-9]{1,6}$ ]] || {
    echo 'ERROR: Path IDs must be nonnegative integers.' >&2; exit 2;
}
start="$((10#$1))"
end_arg="${2:-$1}"
end="$((10#$end_arg))"
(( start <= end )) || { echo 'ERROR: START must not exceed END.' >&2; exit 2; }
suffix="${3:-}"
[[ -z "$suffix" || "$suffix" =~ ^[a-zA-Z0-9_-]+$ ]] || {
    echo 'ERROR: Invalid filename suffix.' >&2; exit 2;
}
cd -- "$WS"
output_log="$(mktemp "${TMPDIR:-/tmp}/arm_bot_dataset.XXXXXXXX")"
trap 'rm -f -- "$output_log"' EXIT

for ((id = start; id <= end; id++)); do
    printf -v padded '%03d' "$id"
    csv="src/scripts/Joint_states/path_${padded}_joint_states${suffix:+_$suffix}.csv"
    [[ -f "$csv" ]] || { echo "ERROR: CSV not found: $csv" >&2; exit 1; }
    python3 src/scripts/torque_publisher.py --csv-path "$csv" 2>&1 | tee -a "$output_log"
    sleep 3
done

mapfile -t generated_log_files < <(sed -n 's/.*File saved: \(.*\.csv\).*/\1/p' "$output_log")
(( ${#generated_log_files[@]} )) || { echo 'ERROR: No generated log files found.' >&2; exit 1; }
sleep 2
for log_file in "${generated_log_files[@]}"; do
    filename="$(basename -- "$log_file")"
    [[ "$filename" =~ path_([0-9]+)_log_ ]] || { echo "ERROR: Unrecognized log filename: $filename" >&2; exit 1; }
    dataset="src/scripts/Joint_states/path_${BASH_REMATCH[1]}_joint_states${suffix:+_$suffix}.csv"
    python3 src/scripts/plot_comparison.py "$dataset" "$log_file" --allinone
done
