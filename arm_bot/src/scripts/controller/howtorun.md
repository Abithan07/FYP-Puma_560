# Option 1: From the dnn_controller directory
## Terminal at 
cd /home/priyankan/Desktop/FYP-Puma_560/arm_bot/src/scripts/controller/dnn_controller

```bash 
python3 dnn_controller.py --target 150 0 90 \
  --delan-model /home/priyankan/Desktop/FYP-Puma_560/DNN_test/fyp_jax_delan_50.jax \
  --gru-model /home/priyankan/Desktop/FYP-Puma_560/DNN_test/best_GRUResidual.pt \
  --scaler /home/priyankan/Desktop/FYP-Puma_560/DNN_test/feature_scaler.pkl
```


# Option 2: From arm_bot directory with module syntax
cd /home/priyankan/Desktop/FYP-Puma_560/arm_bot 

```bash
python3 -m src.scripts.controller.dnn_controller.dnn_controller --target 150 0 90 \
  --delan-model /home/priyankan/Desktop/FYP-Puma_560/DNN_test/fyp_jax_delan_50.jax \
  --gru-model /home/priyankan/Desktop/FYP-Puma_560/DNN_test/best_GRUResidual.pt \
  --scaler /home/priyankan/Desktop/FYP-Puma_560/DNN_test/feature_scaler.pkl
```

```bash
python3 src/scripts/controller/pid_controller/pid_controller.py --target 150 0 90
```

