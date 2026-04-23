# ============================================================================
# HYBRID INFERENCE PIPELINE: DeLaN + GRU
# ============================================================================
import os
import torch
import torch.nn as nn
import pandas as pd
import numpy as np
import dill as pickle
import jax
import jax.numpy as jnp
import haiku as hk
import functools
import time
from sklearn.preprocessing import StandardScaler

# ── 1. Setup & Memory Fix ──────────────────────────────────────────────
# Prevent JAX from hogging GPU memory
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
os.environ["XLA_PYTHON_CLIENT_ALLOCATOR"] = "platform"

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"✅ Using device: {device}")

# ============================================================================
# ⚙️ USER CONFIGURATION (Change these paths!)
# ============================================================================
CSV_INPUT_PATH = "my_new_trajectory.csv"      # Your raw data
CSV_OUTPUT_PATH = "predicted_torques.csv"     # Where to save results

DELAN_MODEL_PATH = "TrainedModels/fyp_jax_delan_50.jax"
GRU_MODEL_PATH = "ResidualModels/best_GRUResidual.pt"

# Update these column names to match exactly what is inside your CSV file!
Q_COLS =   ['q_0', 'q_1', 'q_2']
QD_COLS =  ['qd_0', 'qd_1', 'qd_2']
QDD_COLS = ['qdd_0', 'qdd_1', 'qdd_2']

SEQ_LEN = 128
N_DOF = 3
INPUT_DIM = 12
HIDDEN_DIM = 64
N_LAYERS = 4

# ── 2. Load DeLaN Model ────────────────────────────────────────────────
print("📂 Loading DeLaN...")
with open(DELAN_MODEL_PATH, 'rb') as f:
    saved = pickle.load(f)
params = saved["params"]
hyper = saved["hyper"]
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
    e_kin = kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift)
    e_pot = potential_energy_fn(q, shape, activation).squeeze()
    return e_kin - e_pot

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
    structured_lagrangian_fn, n_dof=N_DOF,
    shape=(hyper['n_width'],) * hyper['n_depth'],
    activation=activations[hyper['activation']],
    epsilon=hyper['diagonal_epsilon'], shift=hyper['diagonal_shift'],
))
lagrangian = lagrangian_fn.apply
delan_model = jax.jit(functools.partial(dynamics_model, lagrangian=lagrangian, n_dof=N_DOF))

# ── 3. Load GRU Model ──────────────────────────────────────────────────
class GRUResidual(nn.Module):
    def __init__(self):
        super().__init__()
        self.gru = nn.GRU(input_size=INPUT_DIM, hidden_size=HIDDEN_DIM, num_layers=N_LAYERS, batch_first=True, dropout=0.1)
        self.fc = nn.Linear(HIDDEN_DIM, N_DOF)
    def forward(self, x):
        h, _ = self.gru(x)
        return self.fc(h[:, -1, :])

print("📂 Loading GRU...")
gru_model = GRUResidual().to(device)
gru_model.load_state_dict(torch.load(GRU_MODEL_PATH, map_location=device))
gru_model.eval()

# ── 4. Load & Process CSV Data ─────────────────────────────────────────
print(f"📄 Reading CSV: {CSV_INPUT_PATH}...")
df = pd.read_csv(CSV_INPUT_PATH)

q = df[Q_COLS].values
qd = df[QD_COLS].values
qdd = df[QDD_COLS].values

# Step A: Run DeLaN Baseline
print("🔄 Running DeLaN physics baseline...")
t0 = time.time()
tau_delan = np.array(delan_model(params, None, jnp.array(q), jnp.array(qd), jnp.array(qdd), jnp.zeros_like(q))[1])
print(f"✅ DeLaN finished in {time.time()-t0:.3f}s")

# Step B: Prepare Features for GRU
features = np.concatenate([q, qd, qdd, tau_delan], axis=1)

# *NOTE: For production, you should load the exact StandardScaler from training. 
# Here, we fit a new one on the input data for quick inference.
scaler = StandardScaler()
features_norm = scaler.fit_transform(features)

# Step C: Create Sliding Windows
print("🔄 Creating sliding windows and running GRU...")
X_win = [features_norm[i : i + SEQ_LEN] for i in range(len(features_norm) - SEQ_LEN + 1)]
X_win_tensor = torch.FloatTensor(np.array(X_win)).to(device)

# Step D: Predict Residuals
with torch.no_grad():
    tau_residual = gru_model(X_win_tensor).cpu().numpy()

# ── 5. Combine and Save ────────────────────────────────────────────────
# Because the GRU needs 128 timesteps of history to make its first prediction, 
# the first 127 timesteps only rely on DeLaN. We combine them here:

final_torques = np.copy(tau_delan)
final_torques[SEQ_LEN - 1:] += tau_residual # Add GRU corrections where history is available

# Save to new CSV
output_df = df.copy()
output_df['tau_pred_j1'] = final_torques[:, 0]
output_df['tau_pred_j2'] = final_torques[:, 1]
output_df['tau_pred_j3'] = final_torques[:, 2]

output_df.to_csv(CSV_OUTPUT_PATH, index=False)
print(f"\n🎉 Success! Final hybrid torques saved to: {CSV_OUTPUT_PATH}")