python3 src/scripts/controller/dnn_controller.py --target 0 45 90 --delan-model /path/to/fyp_jax_delan_50.jax --gru-model /path/to/best_GRUResidual.pt --scaler /path/to/feature_scaler.pkl


python3 /data/ros2/ros2_ws2/arm_bot/src/scripts/controller/pid_controller/pid_controller.py \
  --target 150 0 90