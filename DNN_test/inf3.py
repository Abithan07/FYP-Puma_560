# ============================================================================
# HYBRID INFERENCE PIPELINE: DeLaN + GRU (Row-wise CSV, Plotting)
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
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler

# ── 1. Setup & GPU Fix ──────────────────────────────────────────────
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
os.environ["XLA_PYTHON_CLIENT_ALLOCATOR"] = "platform"

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# ============================================================================
# ⚙️ USER CONFIGURATION
# ============================================================================
CSV_INPUT_PATH = "my_new_trajectory.csv"      # input CSV (row-wise)
CSV_OUTPUT_PATH = "predicted_torques.csv"     # output CSV

DELAN_MODEL_PATH = "fyp_jax_delan_50.jax"
GRU_MODEL_PATH = "best_GRUResidual.pt"

SEQ_LEN = 128
N_DOF = 3
INPUT_DIM = 12
HIDDEN_DIM = 64
N_LAYERS = 4

# Input CSV row names
ROW_NAMES = ['t', 'q1', 'q2', 'q3', 'qd1', 'qd2', 'qd3', 'qdd1', 'qdd2', 'qdd3']

# ============================================================================
# 2. Load DeLaN Model
# ============================================================================
print("Loading DeLaN model...")
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
    return 0.5 * jnp.dot(qd, jnp.dot(mass_mat, qd))

def potential_energy_fn(q, shape, activation):
    net = hk.nets.MLP(output_sizes=shape + (1,), activation=activation, name="potential_energy")
    z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
    return net(z)

def structured_lagrangian_fn(q, qd, n_dof, shape, activation, epsilon, shift):
    return kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift) - potential_energy_fn(q, shape, activation).squeeze()

def dynamics_model(params, key, q, qd, qdd, tau, lagrangian, n_dof):
    argnums = [2,3]
    vmap_dim = (None, None, 0, 0)
    batch_matmul = jax.vmap(jnp.matmul, (0,0))
    lag_val_grad = jax.value_and_grad(lagrangian, argnums=argnums)
    L, (dLdq, dLdqd) = jax.vmap(lag_val_grad, vmap_dim)(params, key, q, qd)
    lag_hess = jax.hessian(lagrangian, argnums=argnums)
    _, (d2L_dqddq, d2Ld2qd) = jax.vmap(lag_hess, vmap_dim)(params, key, q, qd)
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

# ============================================================================
# 3. Load GRU Model
# ============================================================================
class GRUResidual(nn.Module):
    def __init__(self):
        super().__init__()
        self.gru = nn.GRU(input_size=INPUT_DIM, hidden_size=HIDDEN_DIM, num_layers=N_LAYERS, batch_first=True, dropout=0.1)
        self.fc = nn.Linear(HIDDEN_DIM, N_DOF)
    def forward(self, x):
        h,_ = self.gru(x)
        return self.fc(h[:,-1,:])

print("Loading GRU model...")
gru_model = GRUResidual().to(device)
gru_model.load_state_dict(torch.load(GRU_MODEL_PATH, map_location=device))
gru_model.eval()

# ============================================================================
# 4. Load CSV & Prepare Features
# ============================================================================
print(f"Reading CSV: {CSV_INPUT_PATH}...")
df = pd.read_csv(CSV_INPUT_PATH, header=None)
df.columns = ROW_NAMES

q = df[['q1','q2','q3']].values
qd = df[['qd1','qd2','qd3']].values
qdd = df[['qdd1','qdd2','qdd3']].values

print("🔄 Running DeLaN baseline...")
t0 = time.time()
tau_delan = np.array(delan_model(params, None, jnp.array(q), jnp.array(qd), jnp.array(qdd), jnp.zeros_like(q))[1])
print(f"✅ DeLaN finished in {time.time()-t0:.3f}s")

features = np.concatenate([q, qd, qdd, tau_delan], axis=1)
scaler = StandardScaler()
features_norm = scaler.fit_transform(features)

# Sliding windows
X_win = [features_norm[i:i+SEQ_LEN] for i in range(len(features_norm)-SEQ_LEN+1)]
X_win_tensor = torch.FloatTensor(np.array(X_win)).to(device)

# GRU predictions
print("Running GRU residuals...")
with torch.no_grad():
    tau_residual = gru_model(X_win_tensor).cpu().numpy()

# Combine final torques
final_torques = np.copy(tau_delan)
final_torques[SEQ_LEN-1:] += tau_residual

# ============================================================================
# 5. Save CSV in ROW-WISE FORMAT
# ============================================================================
output_df = pd.DataFrame(index=df.index)
output_df['t'] = df['t']
output_df['q1'] = df['q1']
output_df['q2'] = df['q2']
output_df['q3'] = df['q3']
output_df['qd1'] = df['qd1']
output_df['qd2'] = df['qd2']
output_df['qd3'] = df['qd3']
output_df['qdd1'] = df['qdd1']
output_df['qdd2'] = df['qdd2']
output_df['qdd3'] = df['qdd3']
output_df['tau1'] = final_torques[:,0]
output_df['tau2'] = final_torques[:,1]
output_df['tau3'] = final_torques[:,2]

output_df.to_csv(CSV_OUTPUT_PATH, index=False)
print(f"\n Success! Hybrid torques saved to: {CSV_OUTPUT_PATH}")

# ============================================================================
# 6. Plot Torques & Errors (All 3 joints)
# ============================================================================
time_axis = df['t'].values
plt.figure(figsize=(16,8))

for j in range(3):
    plt.subplot(3,2,2*j+1)
    plt.plot(time_axis, final_torques[:,j], 'b-', label='DeLaN+GRU')
    plt.plot(time_axis, tau_delan[:,j], 'r--', label='DeLaN Only', alpha=0.7)
    plt.title(f'Joint {j+1} Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3,2,2*j+2)
    err_delan = np.abs(tau_delan[:,j] - final_torques[:,j])
    err_gru = np.abs(final_torques[:,j] - tau_delan[:,j])
    plt.plot(time_axis, err_delan, 'r-', label='DeLaN Error', alpha=0.7)
    plt.plot(time_axis, err_gru, 'b-', label='GRU Error', alpha=0.8)
    plt.title(f'Joint {j+1} Absolute Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (Nm)')
    plt.legend()
    plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
print("\nPlotting complete!")