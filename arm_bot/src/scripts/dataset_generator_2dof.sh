#!/bin/bash

generated_log_files=()

if [ $# -eq 2 ]; then

  echo "You provided the following arguments: $@"

#   convert the arguments to integers from strings
  start=$1
  end=$2

  for i in $(seq $start $end); do

    # add a zero padding to the number if it's less than 10
    if [ $i -lt 10 ]; then
      i="0$i"
    fi

    echo "Running on file src/scripts/joint_states_2dof/path_0${i}_joint_states.csv"
    python3 src/scripts/torque_publisher_2dof.py --csv-path src/scripts/joint_states_2dof/path_0${i}_joint_states.csv 2>&1 | tee -a /tmp/torque_output.log

    sleep 3
  done

elif [ $# -eq 1 ]; then

  # add a zero padding to the number if it's less than 10
  if [ $1 -lt 10 ]; then
    i="0$1"
  else
    i="$1"
  fi

  echo "Running on file src/scripts/joint_states_2dof/path_0${i}_joint_states.csv"
  python3 src/scripts/torque_publisher_2dof.py --csv-path src/scripts/joint_states_2dof/path_0${i}_joint_states.csv 2>&1 | tee -a /tmp/torque_output.log
  sleep 3
else
  echo "Invalid arguments provided. You can give either one or two arguments."
fi

# check if the output file exists
if [ -f /tmp/torque_output.log ]; then
  # Extract CSV filenames from the saved output (handles multiple matches)
  mapfile -t csv_files < <(grep -oP 'File saved: \K.*\.csv' /tmp/torque_output.log)

  generated_log_files+=("${csv_files[@]}")

  rm /tmp/torque_output.log

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
  for file in "${generated_log_files[@]}"; do
    echo "$file"
  done
  echo "======================================================================"
  echo ""

  for i in $(seq $start $end); do

    j=$((i - start))

    # add a zero padding to the number if it's less than 10
    if [ $i -lt 10 ]; then
      i="0$i"
    fi

    echo "Generating comparison plot for path_0${i}_joint_states.csv"
    echo "python3 /data/ros2/ros2_ws2/arm_bot/src/scripts/plot_comparison.py /data/ros2/ros2_ws2/arm_bot/src/scripts/joint_states_2dof/path_0${i}_joint_states.csv ${generated_log_files[j]} 2>&1 | tee -a /tmp/plot_output.log"
    sleep 2
    python3 /data/ros2/ros2_ws2/arm_bot/src/scripts/plot_comparison.py /data/ros2/ros2_ws2/arm_bot/src/scripts/joint_states_2dof/path_0${i}_joint_states.csv ${generated_log_files[j]} 2>&1 | tee -a /tmp/plot_output.log
  done
fi