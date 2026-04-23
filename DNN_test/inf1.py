# ============================================================================
# FULL INFERENCE + PLOTS: ROW-BASED CSV → DeLaN → GRU → FINAL CSV + GRAPHS
# ============================================================================

import os
import dill as pickle
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from sklearn.preprocessing import StandardScaler
import jax
import jax.numpy as jnp
import haiku as hk
import functools
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------
# 🔹 USER CONFIG
# ----------------------------------------------------------------------------
CSV_INPUT_PATH = "input_data.csv"
DELAN_PATH = "TrainedModels/fyp_jax_delan_50.jax"
GRU_MODEL_PATH = "ResidualModels/best_GRUResidual.pt"
SCALER_PATH = "scaler.pkl"
OUTPUT_CSV_PATH = "final_output.csv"
PLOT_PATH = "torque_plot.png"

SEQ_LEN = 128
N_DOF = 3
INPUT_DIM = N_DOF*4
HIDDEN_DIM = 64
N_LAYERS = 4
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ----------------------------------------------------------------------------
# 🔹 1. LOAD CSV (ROW DATA)
# ----------------------------------------------------------------------------
data = pd.read_csv(CSV_INPUT_PATH, header=None).values  # shape (10, N)
data = data.T  # now rows = timesteps

time = data[:,0]
q = data[:,1:4]
qd = data[:,4:7]
qdd = data[:,7:10]

# ----------------------------------------------------------------------------
# 🔹 2. LOAD DeLaN
# ----------------------------------------------------------------------------
with open(DELAN_PATH, 'rb') as f:
    saved = pickle.load(f)

params = saved["params"]
hyper = saved["hyper"]
activations = {'tanh': jnp.tanh, 'softplus': jax.nn.softplus}

def mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift):
    n_output = int((n_dof**2 + n_dof)/2)
    idx_diag = np.arange(n_dof)+1
    idx_diag = (idx_diag*(idx_diag+1)//2-1).astype(int)
    idx_tril = np.setdiff1d(np.arange(n_output), idx_diag)
    cat_idx = np.hstack((idx_diag, idx_tril))
    idx = np.arange(cat_idx.size)[np.argsort(cat_idx)]
    mat_idx = np.tril_indices(n_dof)

    net = hk.nets.MLP(output_sizes=shape + (n_output,), activation=activation, name="mass_matrix")
    z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
    l_diag, l_off = jnp.split(net(z), [n_dof], axis=-1)
    l_diag = jax.nn.softplus(l_diag + shift) + epsilon
    vec = jnp.concatenate((l_diag, l_off), axis=-1)[..., idx]
    L = jnp.zeros((n_dof, n_dof))
    L = L.at[mat_idx].set(vec[:])
    return jnp.matmul(L, L.T)

def kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift):
    M = mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift)
    return 0.5 * jnp.dot(qd, jnp.dot(M, qd))

def potential_energy_fn(q, shape, activation):
    net = hk.nets.MLP(output_sizes=shape+(1,), activation=activation, name="potential_energy")
    z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
    return net(z)

def structured_lagrangian_fn(q, qd, n_dof, shape, activation, epsilon, shift):
    return kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift) - \
           potential_energy_fn(q, shape, activation).squeeze()

def dynamics_model(params, key, q, qd, qdd, tau, lagrangian, n_dof):
    lagrangian_value_and_grad = jax.value_and_grad(lagrangian, argnums=[2,3])
    L, (dLdq, dLdqd) = jax.vmap(lagrangian_value_and_grad, (None,None,0,0))(params, key, q, qd)
    lagrangian_hessian = jax.hessian(lagrangian, argnums=[2,3])
    (_, (d2L_dqddq, d2Ld2qd)) = jax.vmap(lagrangian_hessian, (None,None,0,0))(params, key, q, qd)
    tau_pred = jax.vmap(jnp.matmul)(d2Ld2qd, qdd) + jax.vmap(jnp.matmul)(d2L_dqddq, qd) - dLdq
    return None, tau_pred, None, None

lagrangian_fn = hk.transform(functools.partial(
    structured_lagrangian_fn,
    n_dof=N_DOF,
    shape=(hyper['n_width'],)*hyper['n_depth'],
    activation=activations[hyper['activation']],
    epsilon=hyper['diagonal_epsilon'],
    shift=hyper['diagonal_shift']
))
lagrangian = lagrangian_fn.apply
delan_model = jax.jit(functools.partial(dynamics_model, lagrangian=lagrangian, n_dof=N_DOF))

# ----------------------------------------------------------------------------
# 🔹 3. RUN DeLaN → tau_delan
# ----------------------------------------------------------------------------
tau_delan = np.array(delan_model(params, None,
                                 jnp.array(q),
                                 jnp.array(qd),
                                 jnp.array(qdd),
                                 jnp.zeros_like(q))[1])

# ----------------------------------------------------------------------------
# 🔹 4. BUILD FEATURES
# ----------------------------------------------------------------------------
features = np.concatenate([q, qd, qdd, tau_delan], axis=1)

# ----------------------------------------------------------------------------
# 🔹 5. LOAD SCALER & NORMALIZE
# ----------------------------------------------------------------------------
with open(SCALER_PATH, 'rb') as f:
    scaler = pickle.load(f)

features_norm = scaler.transform(features)

# ----------------------------------------------------------------------------
# 🔹 6. CREATE WINDOWS
# ----------------------------------------------------------------------------
X_windows = np.array([features_norm[i:i+SEQ_LEN] for i in range(0, len(features_norm)-SEQ_LEN+1)])

# ----------------------------------------------------------------------------
# 🔹 7. LOAD GRU MODEL
# ----------------------------------------------------------------------------
class GRUResidual(nn.Module):
    def __init__(self):
        super().__init__()
        self.gru = nn.GRU(INPUT_DIM, HIDDEN_DIM, N_LAYERS, batch_first=True)
        self.fc = nn.Linear(HIDDEN_DIM, N_DOF)
    def forward(self, x):
        h,_ = self.gru(x)
        return self.fc(h[:, -1, :])

model = GRUResidual().to(device)
model.load_state_dict(torch.load(GRU_MODEL_PATH, map_location=device))
model.eval()

# ----------------------------------------------------------------------------
# 🔹 8. PREDICT RESIDUALS
# ----------------------------------------------------------------------------
with torch.no_grad():
    preds = model(torch.FloatTensor(X_windows).to(device)).cpu().numpy()

# ----------------------------------------------------------------------------
# 🔹 9. FINAL TORQUE
# ----------------------------------------------------------------------------
tau_delan_last = tau_delan[SEQ_LEN-1:]
tau_final = preds + tau_delan_last

# ----------------------------------------------------------------------------
# 🔹 10. SAVE OUTPUT CSV (ROW DATA)
# ----------------------------------------------------------------------------
output_data = np.vstack([
    time[SEQ_LEN-1:],       # r1
    q[SEQ_LEN-1:,0],        # r2
    q[SEQ_LEN-1:,1],        # r3
    q[SEQ_LEN-1:,2],        # r4
    qd[SEQ_LEN-1:,0],       # r5
    qd[SEQ_LEN-1:,1],       # r6
    qd[SEQ_LEN-1:,2],       # r7
    qdd[SEQ_LEN-1:,0],      # r8
    qdd[SEQ_LEN-1:,1],      # r9
    qdd[SEQ_LEN-1:,2],      # r10
    tau_final[:,0],         # r11
    tau_final[:,1],         # r12
    tau_final[:,2],         # r13
])
pd.DataFrame(output_data).to_csv(OUTPUT_CSV_PATH, index=False, header=False)
print(f"✅ CSV saved: {OUTPUT_CSV_PATH}")

# ----------------------------------------------------------------------------
# 🔹 11. PLOT TORQUES & ERRORS
# ----------------------------------------------------------------------------
tau_delan_err = np.abs(tau_delan_last - tau_delan_last)  # just for plot style
gru_err = np.abs(tau_final - tau_delan_last)

time_axis = time[SEQ_LEN-1:]

fig, axes = plt.subplots(3,2, figsize=(18,10))
fig.suptitle("Torque Tracking & Residual Error", fontsize=16)

for j in range(3):
    # Left: torque plot
    ax = axes[j,0]
    ax.plot(time_axis, tau_delan_last[:,j], 'r--', label="DeLaN Only")
    ax.plot(time_axis, tau_final[:,j], 'b-', label="DeLaN + GRU")
    ax.set_ylabel(f"Joint {j+1} Torque (Nm)")
    ax.grid(True, alpha=0.3)
    if j==0: ax.legend()
    if j==2: ax.set_xlabel("Time (s)")
    
    # Right: absolute error plot
    ax = axes[j,1]
    ax.plot(time_axis, tau_delan_err[:,j], 'r-', alpha=0.7, label="DeLaN Error")
    ax.plot(time_axis, gru_err[:,j], 'b-', alpha=0.8, label="GRU Error")
    ax.fill_between(time_axis, 0, tau_delan_err[:,j], color='red', alpha=0.1)
    ax.fill_between(time_axis, 0, gru_err[:,j], color='blue', alpha=0.2)
    ax.set_ylabel(f"Joint {j+1} |Error|")
    ax.grid(True, alpha=0.3)
    if j==0: ax.legend()
    if j==2: ax.set_xlabel("Time (s)")

plt.tight_layout()
plt.savefig(PLOT_PATH, dpi=150)
plt.show()
print(f"✅ Plot saved: {PLOT_PATH}")