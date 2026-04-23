"""
hybrid_inference.py

Standalone inference pipeline for the DeLaN + GRU hybrid torque controller.
Takes raw kinematics (CSV), applies physics baseline, adds GRU friction correction,
and outputs the final predicted torques.
"""

import os
import time
import torch
import torch.nn as nn
import pandas as pd
import numpy as np
import dill as pickle
import joblib
import jax
import jax.numpy as jnp
import haiku as hk
import functools
import matplotlib.pyplot as plt

# ============================================================================
# ⚙️ 1. USER CONFIGURATION (Update paths & column names if needed)
# ============================================================================
# ... (around line 20)
CSV_INPUT_PATH = "/home/priyankan/Desktop/FYP_DNN_CTC/Data/Trajectory/path_461_trajectories.csv"

# --- Auto-generate output path ---
output_dir = "Data"
base_filename = os.path.basename(CSV_INPUT_PATH)
name, ext = os.path.splitext(base_filename)
name_out = name.replace("_trajectories", "_joint_states")
CSV_OUTPUT_PATH = os.path.join(output_dir, f"{name_out}{ext}")

# Paths to your trained models (from residual_training.py)
DELAN_MODEL_PATH = "fyp_jax_delan_50.jax"
GRU_MODEL_PATH   = "best_GRUResidual.pt"
SCALER_PATH      = "feature_scaler.pkl"

# Exact column headers inside your CSV
Q_COLS   = ['dp1', 'dp2', 'dp3']
QD_COLS  = ['dv1', 'dv2', 'dv3']
QDD_COLS = ['da1', 'da2', 'da3']
# TAU_COLS = ['tau1', 'tau2', 'tau3']

# Architecture details (Must match training exactly)
SEQ_LEN = 128
N_DOF = 3
INPUT_DIM = 12
HIDDEN_DIM = 64
N_LAYERS = 4

# ============================================================================
# 🛠️ 2. ENVIRONMENT SETUP
# ============================================================================
# CRITICAL: Prevent JAX from reserving 100% of GPU memory
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
os.environ["XLA_PYTHON_CLIENT_ALLOCATOR"] = "platform"

# Auto-detect device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("="*70)
print(f"🚀 HYBRID INFERENCE ENGINE STARTED (Using {device.type.upper()})")
print("="*70)

# ============================================================================
# 🧠 3. RECONSTRUCT DeLaN (PHYSICS BASELINE)
# ============================================================================
print("📂 Loading DeLaN Physics Model...")
with open(DELAN_MODEL_PATH, 'rb') as f:
    saved = pickle.load(f)
params, hyper = saved["params"], saved["hyper"]
activations = {'tanh': jnp.tanh, 'softplus': jax.nn.softplus}

def mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift):
    n_output = int((n_dof ** 2 + n_dof) / 2)
    idx_diag = np.arange(n_dof, dtype=int) + 1
    idx_diag = (idx_diag * (idx_diag + 1) / 2 - 1).astype(int)
    idx_tril = np.setdiff1d(np.arange(n_output), idx_diag)
    cat_idx = np.hstack((idx_diag, idx_tril))
    idx = np.arange(cat_idx.size)[np.argsort(cat_idx)]
    mat_idx = np.tril_indices(n_dof)
    
    net = hk.nets.MLP(output_sizes=shape + (n_output,), activation=activation, name="mass_matrix")
    z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
    l_diagonal, l_off_diagonal = jnp.split(net(z), [n_dof,], axis=-1)
    l_diagonal = jax.nn.softplus(l_diagonal + shift) + epsilon
    vec_lower_triangular = jnp.concatenate((l_diagonal, l_off_diagonal), axis=-1)[..., idx]
    triangular_mat = jnp.zeros((n_dof, n_dof))
    triangular_mat = triangular_mat.at[mat_idx].set(vec_lower_triangular[:])
    return jnp.matmul(triangular_mat, triangular_mat.transpose())

def kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift):
    mass_mat = mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift)
    return 1. / 2. * jnp.dot(qd, jnp.dot(mass_mat, qd))

def potential_energy_fn(q, shape, activation):
    net = hk.nets.MLP(output_sizes=shape + (1,), activation=activation, name="potential_energy")
    z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
    return net(z)

def structured_lagrangian_fn(q, qd, n_dof, shape, activation, epsilon, shift):
    return kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift) - potential_energy_fn(q, shape, activation).squeeze()

def dynamics_model(params, key, q, qd, qdd, tau, lagrangian, n_dof):
    argnums = [2, 3]
    vmap_dim = (None, None, 0, 0)
    batch_matmul = jax.vmap(jnp.matmul, (0, 0))
    lagrangian_value_and_grad = jax.value_and_grad(lagrangian, argnums=argnums)
    L, (dLdq, dLdqd) = jax.vmap(lagrangian_value_and_grad, vmap_dim)(params, key, q, qd)
    lagrangian_hessian = jax.hessian(lagrangian, argnums=argnums)
    (_, (d2L_dqddq, d2Ld2qd)) = jax.vmap(lagrangian_hessian, vmap_dim)(params, key, q, qd)
    tau_pred = batch_matmul(d2Ld2qd, qdd) + batch_matmul(d2L_dqddq, qd) - dLdq
    return None, tau_pred, None, None

lagrangian_fn = hk.transform(functools.partial(
    structured_lagrangian_fn, n_dof=N_DOF, shape=(hyper['n_width'],) * hyper['n_depth'],
    activation=activations[hyper['activation']], epsilon=hyper['diagonal_epsilon'], shift=hyper['diagonal_shift']
))
lagrangian = lagrangian_fn.apply
delan_model = jax.jit(functools.partial(dynamics_model, lagrangian=lagrangian, n_dof=N_DOF))

# ============================================================================
# 🤖 4. RECONSTRUCT GRU (RESIDUAL NETWORK)
# ============================================================================
print("📂 Loading GRU Residual Model & Scaler...")
class GRUResidual(nn.Module):
    def __init__(self):
        super().__init__()
        self.gru = nn.GRU(input_size=INPUT_DIM, hidden_size=HIDDEN_DIM, num_layers=N_LAYERS, batch_first=True, dropout=0.1)
        self.fc = nn.Linear(HIDDEN_DIM, N_DOF)
    def forward(self, x):
        h, _ = self.gru(x)
        return self.fc(h[:, -1, :])

gru_model = GRUResidual().to(device)
gru_model.load_state_dict(torch.load(GRU_MODEL_PATH, map_location=device))
gru_model.eval()

# Load Training Scaler
scaler = joblib.load(SCALER_PATH)

# ============================================================================
# ⚙️ 5. PIPELINE EXECUTION
# ============================================================================
if not os.path.exists(CSV_INPUT_PATH):
    raise FileNotFoundError(f"❌ Input CSV not found: {CSV_INPUT_PATH}")

print(f"\n📄 Reading data from: {CSV_INPUT_PATH}")
# --- Robust CSV Reading Method ---
# 1. Read the entire file without assuming any headers or index
temp_df = pd.read_csv(CSV_INPUT_PATH, header=None)

# 2. Extract the first column to be used as headers. Clean them up.
headers = temp_df.iloc[:, 0].str.strip().tolist()

# 3. Get the data (all columns except the first), drop the old header column, and transpose.
df = temp_df.iloc[:, 1:].T

# 4. Assign the cleaned headers to the transposed DataFrame.
df.columns = headers

print(f"   ↳ Data shape: {df.shape} (transposed from row format with headers)")


if len(df) < SEQ_LEN:
    raise ValueError(f"❌ Data too short! GRU needs at least {SEQ_LEN} rows of history.")

# Check if all required columns exist
required_cols = ['t'] + Q_COLS + QD_COLS + QDD_COLS
# required_cols = ['t'] + Q_COLS + QD_COLS + QDD_COLS + TAU_COLS
missing_cols = [col for col in required_cols if col not in df.columns]
if missing_cols:
    raise ValueError(f"❌ Missing required data columns in CSV: {missing_cols}")

q   = df[Q_COLS].values
qd  = df[QD_COLS].values
qdd = df[QDD_COLS].values
t   = df['t'].values
# tau = df[TAU_COLS].values

# --- STEP A: DeLaN ---
print("🔄 1/3: Running rigid-body physics (DeLaN)...")
t0 = time.time()
tau_delan = np.array(delan_model(params, None, jnp.array(q), jnp.array(qd), jnp.array(qdd), jnp.zeros_like(q))[1])
print(f"   ↳ Done in {time.time()-t0:.3f}s")

# --- STEP B: Normalize Features ---
print("🔄 2/3: Normalizing features with saved training scaler...")
features = np.concatenate([q, qd, qdd, tau_delan], axis=1)
features_norm = scaler.transform(features) # ONLY transform!

# --- STEP C: Sliding Windows & GRU ---
print("🔄 3/3: Evaluating sequence memory & friction residuals (GRU)...")
X_win = [features_norm[i : i + SEQ_LEN] for i in range(len(features_norm) - SEQ_LEN + 1)]
X_win_tensor = torch.FloatTensor(np.array(X_win)).to(device)

t1 = time.time()
with torch.no_grad():
    tau_residual = gru_model(X_win_tensor).cpu().numpy()
print(f"   ↳ GRU done in {time.time()-t1:.3f}s")

# ============================================================================
# 💾 6. COMBINE & EXPORT (ROW FORMAT)
# ============================================================================
# Base torque is DeLaN. For indices >= 127 (SEQ_LEN-1), add the GRU's learned correction.
final_torques = np.copy(tau_delan)
final_torques[SEQ_LEN - 1:] += tau_residual 

# Create output dataframe in column format first
output_data = {
    't': t,
    'dp1': q[:, 0],
    'dp2': q[:, 1],
    'dp3': q[:, 2],
    'dv1': qd[:, 0],
    'dv2': qd[:, 1],
    'dv3': qd[:, 2],
    'da1': qdd[:, 0],
    'da2': qdd[:, 1],
    'da3': qdd[:, 2],
    'tau_delan_j1': tau_delan[:, 0],
    'tau_delan_j2': tau_delan[:, 1],
    'tau_delan_j3': tau_delan[:, 2],
    'tau1': final_torques[:, 0], #<-- This is the final output with GRU correction
    'tau2': final_torques[:, 1],
    'tau3': final_torques[:, 2],
}

output_df = pd.DataFrame(output_data)

# Transpose to row format (headers in first column) and save
output_df_transposed = output_df.T
output_df_transposed.to_csv(CSV_OUTPUT_PATH, header=False)

# ============================================================================
# 📊 7. PLOTTING (ALL 3 JOINTS)
# ============================================================================
print("\n📊 Generating plots for all 3 joints...")

# --- Create a directory for plots ---
PLOTS_DIR = "Plots"
os.makedirs(PLOTS_DIR, exist_ok=True)
base_plot_name = os.path.splitext(os.path.basename(CSV_OUTPUT_PATH))[0]

fig, axes = plt.subplots(3, 1, figsize=(14, 10))
fig.suptitle('Hybrid Torque Controller: DeLaN vs Hybrid Predictions', fontsize=16, fontweight='bold')

joints = [('J1', 0), ('J2', 1), ('J3', 2)]

for ax, (joint_name, idx) in zip(axes, joints):
    # Plot DeLaN baseline
    ax.plot(t, tau_delan[:, idx], 'b-', linewidth=1.5, label='DeLaN (Physics)', alpha=0.8)
    
    # Plot Hybrid predictions (with offset for SEQ_LEN warmup)
    ax.plot(t, final_torques[:, idx], 'r--', linewidth=1.5, label='Hybrid (DeLaN + GRU)', alpha=0.8)

    # Plot Original Ground Truth Torques
    # ax.plot(t, tau[:, idx], 'g:', linewidth=2, label='Original (Ground Truth)', alpha=0.7) # <-- ADD THIS LINE
    
    # Highlight GRU correction region
    ax.axvline(x=t[SEQ_LEN - 1], color='green', linestyle=':', alpha=0.5, label=f'GRU Warmup End (t={t[SEQ_LEN-1]:.3f})')
    
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel(f'Torque {joint_name} (N·m)', fontsize=11)
    ax.set_title(f'Joint {joint_name}: Torque Prediction Comparison', fontsize=12, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
# Save the combined plot to the "Plots" directory
plot_path = os.path.join(PLOTS_DIR, f"{base_plot_name}_comparison_plots.png")
plt.savefig(plot_path, dpi=300, bbox_inches='tight')
print(f"   ↳ Comparison plot saved to: {plot_path}")

# Individual joint plots
for joint_name, idx in joints:
    fig_single, ax_single = plt.subplots(figsize=(12, 6))
    
    ax_single.plot(t, tau_delan[:, idx], 'b-', linewidth=2, label='DeLaN (Physics)', marker='o', markersize=3, markevery=max(1, len(t)//50))
    ax_single.plot(t, final_torques[:, idx], 'r--', linewidth=2, label='Hybrid (DeLaN + GRU)', marker='s', markersize=3, markevery=max(1, len(t)//50))
    # ax_single.plot(t, tau[:, idx], 'g:', linewidth=2, label='Original (Ground Truth)', alpha=0.7)

    ax_single.axvline(x=t[SEQ_LEN - 1], color='green', linestyle=':', linewidth=2, alpha=0.6, label=f'GRU Warmup End')
    ax_single.fill_between(t[:SEQ_LEN], ax_single.get_ylim()[0], ax_single.get_ylim()[1], alpha=0.1, color='gray', label='GRU Warmup Period')
    
    ax_single.set_xlabel('Time (s)', fontsize=12)
    ax_single.set_ylabel(f'Torque (N·m)', fontsize=12)
    ax_single.set_title(f'Joint {joint_name}: Detailed Torque Analysis', fontsize=14, fontweight='bold')
    ax_single.legend(fontsize=11)
    ax_single.grid(True, alpha=0.3)
    
    # Save individual joint plots to the "Plots" directory
    single_plot_path = os.path.join(PLOTS_DIR, f"{base_plot_name}_joint_{joint_name}_detail.png")
    plt.savefig(single_plot_path, dpi=300, bbox_inches='tight')
    print(f"   ↳ {joint_name} detail plot saved to: {single_plot_path}")
    plt.close(fig_single)

plt.close(fig)

# ============================================================================
# 📈 STATISTICS
# ============================================================================
print("\n" + "="*70)
print(f"🎉 SUCCESS! Processed {len(df)} timesteps.")
print(f"📁 Predictions saved to: {CSV_OUTPUT_PATH}")
print(f"📊 Plots saved with prefixes: *_comparison_plots.png, *_joint_*_detail.png")
print("\n📊 Torque Statistics (Hybrid Predictions):")
for joint_name, idx in joints:
    mean_tau = np.mean(final_torques[:, idx])
    std_tau = np.std(final_torques[:, idx])
    max_tau = np.max(np.abs(final_torques[:, idx]))
    print(f"   {joint_name}: μ={mean_tau:7.3f}, σ={std_tau:7.3f}, max|τ|={max_tau:7.3f} N·m")
print("="*70)