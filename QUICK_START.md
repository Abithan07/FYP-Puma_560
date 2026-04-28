# Quick Start Guide: PUMA-560 Trajectory Generation & Inverse Dynamics

This guide will get you started generating PUMA-560 trajectories and computing their inverse dynamics.

## 1. Basic Single Trajectory Workflow

### Step 1: Generate a trajectory
```bash
cd /home/priyankan/Desktop/FYP-Puma_560
python3 trajectory_generator.py 1 --q-end-deg 115.0 0.0 90.0 --t-total 15.0 --path-id 5600
```

Expected output:
```
Generating trajectory 1 / 1 (path ID: 5600)
  Using specified duration T=15.00s (min required: 1.88s)
All trajectories generated successfully.
```

**Output files created:**
- `Dataset/Trajectories/path_5600_traj.csv` - Full trajectory with time, positions, velocities, accelerations
- `Dataset/Angles/path_5600_angles.csv` - Joint angles only
- `Dataset/XYZ/path_5600_xyz.csv` - End-effector positions
- `Dataset/JointStatesAll/all_paths_0.csv` - Path record

### Step 2: Compute inverse dynamics
```bash
python3 inverse_dynamics_fast.py Dataset/Trajectories/path_5600_traj.csv -o Dataset/Joint_states
```

Expected output:
```
Initializing inverse dynamics calculator (fast numerical method)...
Loading trajectory from Dataset/Trajectories/path_5600_traj.csv...
Computing torques for 1501 time steps...
  Progress: 150/1501
  Progress: 300/1501
  ...
  Progress: 1500/1501
Saved results to Dataset/Joint_states/path_5600_joint_states.csv
```

### Step 3: Examine results
```bash
# View output file info
ls -lh Dataset/Joint_states/path_5600_joint_states.csv

# Check first few rows (time, joint positions, torques)
head -15 Dataset/Joint_states/path_5600_joint_states.csv | cut -d',' -f1-4
```

## 2. Batch Processing Multiple Trajectories

### Option A: Generate multiple trajectories then compute dynamics
```bash
# Generate 10 trajectories (IDs 1001-1010)
python3 trajectory_generator.py 10 --start-id 1001

# Compute inverse dynamics for all
python3 inverse_dynamics_fast.py Dataset/Trajectories -o Dataset/Joint_states
```

### Option B: Use the batch processor (all-in-one)
```bash
# Generate 10 trajectories AND compute dynamics in one command
python3 batch_processor.py full --num-paths 10 --start-id 1001
```

### Option C: Process only inverse dynamics for existing trajectories
```bash
# Compute inverse dynamics for all trajectories currently in Dataset/Trajectories
python3 batch_processor.py dynamics
```

## 3. Advanced Examples

### Generate trajectories with custom parameters
```bash
# Custom velocity/acceleration limits
python3 trajectory_generator.py 5 \
    --start-id 2000 \
    --v-max 1.5 \
    --a-max 5.0 \
    --seed 42 \
    --plot
```

### Process specific trajectory file
```bash
# Process just one file
python3 inverse_dynamics_fast.py Dataset/Trajectories/path_5600_traj.csv \
    -o /custom/output/path
```

### Batch processing with custom settings
```bash
python3 batch_processor.py full \
    --num-paths 20 \
    --start-id 3000 \
    --v-max 2.5 \
    --a-max 8.0 \
    --seed 12345
```

## 4. Output Files Structure

After running the pipeline, your Dataset directory will contain:

```
Dataset/
├── Trajectories/           # Generated trajectories
│   ├── path_5600_traj.csv
│   ├── path_5601_traj.csv
│   └── ...
├── Angles/                 # Joint angle data only
│   ├── path_5600_angles.csv
│   └── ...
├── XYZ/                    # End-effector positions
│   ├── path_5600_xyz.csv
│   └── ...
├── Joint_states/           # Computed inverse dynamics (MAIN OUTPUT)
│   ├── path_5600_joint_states.csv
│   ├── path_5601_joint_states.csv
│   └── ...
└── JointStatesAll/         # Path records
    └── all_paths_0.csv
```

## 5. Understanding the Output

Each `*_joint_states.csv` file contains:

| Data | Rows | Description |
|------|------|-------------|
| **Time** | 1 | Timestamp for each sample |
| **Positions** | 3 | Joint angles q1, q2, q3 (radians) |
| **Velocities** | 3 | Joint velocities dq1, dq2, dq3 (rad/s) |
| **Accelerations** | 3 | Joint accelerations ddq1, ddq2, ddq3 (rad/s²) |
| **Torques** | 3 | **Computed torques tau1, tau2, tau3 (N⋅m)** |
| **Inertia** | 3 | M(q)⋅ddq term (inertial effects) |
| **Coriolis** | 3 | C(q,dq)⋅dq term (Coriolis/centripetal) |
| **Gravity** | 3 | G(q) term (gravity effects) |

The torques satisfy the inverse dynamics equation:
```
τ = M + C + G
```

## 6. Key Parameters

### Trajectory Generation
- `--q-end-deg`: End joint angles in degrees (3 values: q1 q2 q3)
- `--t-total`: Total trajectory duration in seconds
- `--path-id`: Unique identifier for this trajectory
- `--v-max`: Maximum joint velocity (rad/s), default 2.0
- `--a-max`: Maximum joint acceleration (rad/s²), default 7.0
- `--seed`: Random seed for reproducibility
- `--plot`: Show 3D plots during generation

### Inverse Dynamics
- Input: Trajectory file (or directory of files)
- Output: Joint states with computed torques
- `-o`: Output directory for results
- `--output-dir-name`: Subdirectory name (default: Joint_states)

## 7. Troubleshooting

### Issue: "No trajectory files found"
**Solution**: Make sure you generated trajectories first:
```bash
python3 trajectory_generator.py 1 --q-end-deg 90 45 180
```

### Issue: Script runs but output file is empty
**Solution**: Check if inverse dynamics computation succeeded. Look for error messages in console.

### Issue: Computation is very slow
**Solution**: You're using the wrong script. Use `inverse_dynamics_fast.py` (recommended):
```bash
# Wrong (slow)
python3 inverse_dynamics_calculator.py ...

# Right (fast)
python3 inverse_dynamics_fast.py ...
```

### Issue: Need to recompute everything
**Solution**: Clean up old files and restart:
```bash
rm -rf Dataset/Trajectories/* Dataset/Joint_states/*
python3 trajectory_generator.py 1
python3 inverse_dynamics_fast.py Dataset/Trajectories
```

## 8. Performance Tips

1. **Use `inverse_dynamics_fast.py`** - It's optimized and ~2x faster than alternatives
2. **Batch process directories** instead of individual files for better throughput
3. **Use `--seed` for reproducibility** when generating trajectories
4. **Reduce trajectory duration** when testing to speed up inverse dynamics
5. **Process on a machine with adequate RAM** for large batches

## 9. Next Steps

1. **Generate dataset**: `python3 trajectory_generator.py 100 --start-id 1000`
2. **Compute dynamics**: `python3 inverse_dynamics_fast.py Dataset/Trajectories`
3. **Analyze results**: Load the Joint_states CSV files in MATLAB, Python, or Excel
4. **Train models**: Use the generated data to train neural networks or other models

## 10. Files Reference

| File | Purpose |
|------|---------|
| `trajectory_generator.py` | Generate PUMA-560 trajectories |
| `inverse_dynamics_fast.py` | **Recommended** inverse dynamics calculator |
| `inverse_dynamics_calculator.py` | Symbolic inverse dynamics (slow but accurate) |
| `inverse_dynamics_calculator_numerical.py` | Numerical inverse dynamics (medium speed) |
| `batch_processor.py` | Unified batch processing script |
| `INVERSE_DYNAMICS_README.md` | Comprehensive technical documentation |
| `QUICK_START.md` | This file |

## Support

For detailed technical documentation, see `INVERSE_DYNAMICS_README.md`

For issues or questions, check the troubleshooting section above.
