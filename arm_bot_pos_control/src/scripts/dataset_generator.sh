#!/bin/bash

generated_log_files=()

if [ $# -eq 3 ]; then

  echo "You provided the following arguments: $@"

#   convert the arguments to integers from strings
  start=$1
  end=$2
  suffix=$3

  for i in $(seq $start $end); do

    # add a zero padding to the number if it's less than 10
    if [ $i -lt 10 ]; then
      i="00$i"
    elif [ $i -lt 100 ]; then
      i="0$i"
    else
      i="$i"
    fi

    echo "Running on file src/scripts/Joint_states/path_${i}_joint_states_${suffix}.csv"
    # python3 src/scripts/position_publisher.py --csv-path src/scripts/Joint_states/path_${i}_joint_states.csv
    python3 src/scripts/position_publisher.py --csv-path src/scripts/Joint_states/path_${i}_joint_states_${suffix}.csv 2>&1 | tee -a /tmp/position_output.log

    sleep 3
  done


elif [ $# -eq 2 ]; then

  echo "You provided the following arguments: $@"

#   convert the arguments to integers from strings
  start=$1
  end=$2

  for i in $(seq $start $end); do

    # add a zero padding to the number if it's less than 10
    if [ $i -lt 10 ]; then
      i="0$i"
    elif [ $i -lt 100 ]; then
      i="0$i"
    else
      i="$i"
    fi

    echo "Running on file src/scripts/Joint_states/path_${i}_joint_states.csv"
    # python3 src/scripts/position_publisher.py --csv-path src/scripts/Joint_states/path_${i}_joint_states.csv
    python3 src/scripts/position_publisher.py --csv-path src/scripts/Joint_states/path_${i}_joint_states.csv 2>&1 | tee -a /tmp/position_output.log

    sleep 3
  done

elif [ $# -eq 1 ]; then

  # add a zero padding to the number if it's less than 10
  if [ $1 -lt 10 ]; then
    i="0$1"
  else
    i="$1"
  fi

  echo "Running on file src/scripts/Joint_states/path_${i}_joint_states.csv"
#   python3 src/scripts/position_publisher.py --csv-path src/scripts/Joint_states/path_${1}_joint_states.csv
  python3 src/scripts/position_publisher.py --csv-path src/scripts/Joint_states/path_${i}_joint_states.csv 2>&1 | tee -a /tmp/position_output.log
  sleep 3
else
  echo "Invalid arguments provided. You can give either one or two arguments."
fi

# check if the output file exists
if [ -f /tmp/position_output.log ]; then
  # Extract CSV filenames from the saved output (handles multiple matches)
  mapfile -t csv_files < <(grep -oP 'File saved: \K.*\.csv' /tmp/position_output.log)

  generated_log_files+=("${csv_files[@]}")

  rm /tmp/position_output.log

else
  echo "No output log found. No files were generated."
fi

sleep 2

if [ ${#generated_log_files[@]} -eq 0 ]; then
  echo "No generated files found. Exiting."
  exit 1
else
  echo ""
  echo "======================================================================"
  echo "Generated log files:"
  echo "======================================================================"

  for log_file in "${generated_log_files[@]}"; do
    # Extract file number from log filename (e.g., path_005_log_1.csv -> 005)
    file_num=$(basename "$log_file" .csv | grep -oP 'path_\K[^_]+')
    
    # Determine dataset file based on number of arguments
    if [ $# -eq 3 ]; then
      dataset_file="src/scripts/Joint_states/path_${file_num}_joint_states_${suffix}.csv"
      echo "Generating comparison plot for path_${file_num}_joint_states_${suffix}.csv"
    else
      dataset_file="src/scripts/Joint_states/path_${file_num}_joint_states.csv"
      echo "Generating comparison plot for path_${file_num}_joint_states.csv"
    fi
    
    sleep 2
    python3 src/scripts/plot_comparison.py "$dataset_file" "$log_file" --allinone
  done
fi

# while true; do spd-say -w 'Run Completed, , ,'; done