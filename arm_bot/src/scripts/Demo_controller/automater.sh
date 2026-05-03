#!/usr/bin/env bash
# =============================================================
# run_traj.sh  —  Wrapper for PUMA-560 Trajectory Generator
#
# Supports two modes:
#
# MODE 1 – Direct (single condition):
#   ./run_traj.sh \
#       --path-id   1 \
#       --q-end     [120,-30,80] \
#       --q-mid     [60,10,110] \
#       --t-total   20 \
#       --num-paths 3
#
# MODE 2 – Batch CSV:
#   ./run_traj.sh --csv conditions.csv
#
# Notes:
#   --q-end  and --q-mid use bracket notation: [v1,v2,v3]  (no spaces)
#   --q-mid, --t-total are optional in direct mode.
#   --csv and --path-id are mutually exclusive.
# =============================================================

set -euo pipefail

# ── defaults ────────────────────────────────────────────────
PATH_ID=""
Q_END=""
Q_MID=""
T_TOTAL="18"
NUM_PATHS="1"
CSV_FILE=""
BASE_DIR="/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/Demo_controller/demo_trajectories"
DT="0.01"
V_MAX="2.0"
A_MAX="7.0"
SCRIPT_PATH="/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/Demo_controller/puma_trajectory_generator.py"
SIMULATION_SCRIPT="/home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/Demo_controller/torque_publisher_dnn.py"

# ── helpers ─────────────────────────────────────────────────
get_latest_traj_csv() {
  local traj_dir="$1"

  if [[ ! -d "$traj_dir" ]]; then
    return 1
  fi

  # Pick the newest generated trajectory file.
  find "$traj_dir" -maxdepth 1 -type f -name 'path_*_traj.csv' -printf '%T@ %p\n' 2>/dev/null \
    | sort -n \
    | tail -n 1 \
    | cut -d' ' -f2-
}

get_next_path_id() {
  local traj_dir="$1"
  local max_id=0

  if [[ ! -d "$traj_dir" ]]; then
    echo "1"
    return 0
  fi

  while IFS= read -r -d '' file; do
    local filename
    filename="$(basename "$file")"
    if [[ "$filename" =~ ^path_([0-9]+)_traj\.csv$ ]]; then
      local id
      id=$((10#${BASH_REMATCH[1]}))
      (( id > max_id )) && max_id="$id"
    fi
  done < <(find "$traj_dir" -maxdepth 1 -type f -name 'path_*_traj.csv' -print0 2>/dev/null)

  echo "$((max_id + 1))"
}

run_simulation_with_csv() {
  local csv_path="$1"

  if [[ ! -f "$SIMULATION_SCRIPT" ]]; then
    echo "ERROR: Simulation script not found: $SIMULATION_SCRIPT" >&2
    echo "       Use --simulation-script to specify its location." >&2
    exit 1
  fi

  if [[ ! -f "$csv_path" ]]; then
    echo "ERROR: Trajectory CSV not found: $csv_path" >&2
    exit 1
  fi

  echo "─────────────────────────────────────────"
  echo "  Running simulation"
  echo "─────────────────────────────────────────"
  echo "  simulation-script : $SIMULATION_SCRIPT"
  echo "  csv-path          : $csv_path"
  echo "─────────────────────────────────────────"

  python3 "$SIMULATION_SCRIPT" --csv-path "$csv_path"
}

# ── help ────────────────────────────────────────────────────
usage() {
  cat <<EOF
Usage: $(basename "$0") [MODE] [OPTIONS]

MODE (pick one):
  --path-id   INT          Direct mode: starting path ID (optional; auto if omitted)
  --csv       FILE         Batch mode: path to conditions CSV file

Direct mode options:
  --q-end     [v1,v2,v3]  End joint angles in degrees       (required)
  --q-mid     [v1,v2,v3]  Waypoint joint angles in degrees  (optional)
  --t-total   FLOAT        Total duration in seconds         (optional, auto if omitted)
  --num-paths INT          Number of trajectories            (default: 1)

Shared options:
  --base-dir  PATH         Output base directory             (default: ./test_output)
  --script    PATH         Path to the Python generator script
  --simulation-script PATH Path to the simulation script
  --dt        FLOAT        Time step (default: 0.01)
  --v-max     FLOAT        Max velocity rad/s (default: 2.0)
  --a-max     FLOAT        Max acceleration rad/s^2 (default: 7.0)
  -h, --help               Show this help message

Examples:
  # Direct — with waypoint and explicit duration:
  $(basename "$0") --path-id 1 --q-end [120,-30,80] --q-mid [60,10,110] --t-total 20 --num-paths 3

  # Direct — no waypoint, auto duration:
  $(basename "$0") --path-id 4 --q-end [100,15,90]

  # Direct — multiple paths, no waypoint:
  $(basename "$0") --path-id 5 --q-end [-140,-45,200] --num-paths 2

  # Direct — auto next path-id from output directory:
  $(basename "$0") --q-end [120,-30,80]

  # Batch CSV:
  $(basename "$0") --csv conditions.csv --base-dir ./output
EOF
  exit 0
}

# ── argument parsing ─────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --path-id)   PATH_ID="$2";    shift 2 ;;
    --q-end)     Q_END="$2";      shift 2 ;;
    --q-mid)     Q_MID="$2";      shift 2 ;;
    --t-total)   T_TOTAL="$2";    shift 2 ;;
    --num-paths) NUM_PATHS="$2";  shift 2 ;;
    --csv)       CSV_FILE="$2";   shift 2 ;;
    --base-dir)  BASE_DIR="$2";   shift 2 ;;
    --script)    SCRIPT_PATH="$2"; shift 2 ;;
    --simulation-script) SIMULATION_SCRIPT="$2"; shift 2 ;;
    --dt)        DT="$2";         shift 2 ;;
    --v-max)     V_MAX="$2";      shift 2 ;;
    --a-max)     A_MAX="$2";      shift 2 ;;
    -h|--help)   usage ;;
    *) echo "ERROR: Unknown option: $1" >&2; usage ;;
  esac
done

# ── validate: mutually exclusive modes ───────────────────────
if [[ -n "$CSV_FILE" && -n "$PATH_ID" ]]; then
  echo "ERROR: --csv and --path-id are mutually exclusive." >&2
  exit 1
fi

# ── validate script exists ───────────────────────────────────
if [[ ! -f "$SCRIPT_PATH" ]]; then
  echo "ERROR: Python script not found: $SCRIPT_PATH" >&2
  echo "       Use --script to specify its location." >&2
  exit 1
fi

# ── CSV batch mode ───────────────────────────────────────────
if [[ -n "$CSV_FILE" ]]; then
  if [[ ! -f "$CSV_FILE" ]]; then
    echo "ERROR: CSV file not found: $CSV_FILE" >&2
    exit 1
  fi

  echo "─────────────────────────────────────────"
  echo "  PUMA-560 Trajectory Generator  [BATCH]"
  echo "─────────────────────────────────────────"
  echo "  csv       : $CSV_FILE"
  echo "  output    : $BASE_DIR"
  echo "─────────────────────────────────────────"

  python3 "$SCRIPT_PATH" \
    --csv      "$CSV_FILE" \
    --base-dir "$BASE_DIR" \
    --dt       "$DT" \
    --v-max    "$V_MAX" \
    --a-max    "$A_MAX"

  TRAJ_DIR="$BASE_DIR/Trajectories"
  GENERATED_TRAJ="$(get_latest_traj_csv "$TRAJ_DIR" || true)"

  if [[ -z "$GENERATED_TRAJ" ]]; then
    echo "ERROR: No trajectory CSV found in: $TRAJ_DIR" >&2
    exit 1
  fi

  run_simulation_with_csv "$GENERATED_TRAJ"
  exit 0
fi

# ── direct mode ──────────────────────────────────────────────

# Validate required direct-mode args
if [[ -z "$Q_END" ]]; then
  echo "ERROR: --q-end is required in direct mode." >&2
  exit 1
fi

if [[ -z "$PATH_ID" ]]; then
  PATH_ID="$(get_next_path_id "$BASE_DIR/Trajectories")"
  echo "INFO: --path-id not provided. Using next available path-id: $PATH_ID"
fi

if ! [[ "$PATH_ID" =~ ^[0-9]+$ ]] || (( PATH_ID < 1 )); then
  echo "ERROR: --path-id must be a positive integer. Got: $PATH_ID" >&2
  exit 1
fi

echo "─────────────────────────────────────────"
echo "  PUMA-560 Trajectory Generator  [DIRECT]"
echo "─────────────────────────────────────────"
echo "  path-id   : $PATH_ID"
echo "  q-end     : $Q_END"
echo "  q-mid     : ${Q_MID:-(none)}"
echo "  t-total   : ${T_TOTAL:-auto}"
echo "  num-paths : $NUM_PATHS"
echo "  output    : $BASE_DIR"
echo "─────────────────────────────────────────"

# Build the python command incrementally so optional args are
# only passed when they were actually provided by the user.
CMD=(
  python3 "$SCRIPT_PATH"
  --path-id   "$PATH_ID"
  --q-end     "$Q_END"
  --num-paths "$NUM_PATHS"
  --base-dir  "$BASE_DIR"
  --dt        "$DT"
  --v-max     "$V_MAX"
  --a-max     "$A_MAX"
)

[[ -n "$Q_MID"    ]] && CMD+=(--q-mid   "$Q_MID")
[[ -n "$T_TOTAL"  ]] && CMD+=(--t-total "$T_TOTAL")

"${CMD[@]}"

TRAJ_DIR="$BASE_DIR/Trajectories"
GENERATED_TRAJ="$(get_latest_traj_csv "$TRAJ_DIR" || true)"

if [[ -z "$GENERATED_TRAJ" ]]; then
  echo "ERROR: No trajectory CSV found in: $TRAJ_DIR" >&2
  exit 1
fi

run_simulation_with_csv "$GENERATED_TRAJ"


