import numpy as np
import numpy.random as random
import dill as pickle
import jax
import jax.numpy as jnp
import haiku as hk
import functools
import time
import optax
import matplotlib as mp
import csv
import os

try:
    mp.use("Qt5Agg")
    mp.rc('text', usetex=False)
    #mp.rcParams['text.latex.preamble'] = r"\usepackage{amsmath}"

    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

except:
    pass

def dynamics_network(q, qd, qdd, n_dof, shape, activation):
    inverse_net = hk.nets.MLP(output_sizes= shape + (n_dof,),
                      activation=activation,
                      name="inverse_model")

    # Apply feature transform
    z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
    tau_pred = inverse_net(jnp.concatenate([z, qd, qdd], axis=-1))
    return tau_pred

def dynamics_model(params, key, q, qd, qdd, black_box_model):
    n_samples = q.shape[0]

    # Compute the inverse model:
    tau_pred = black_box_model(params, key, q, qd, qdd)

    # Compute Hamiltonian & dH/dt:
    H = jnp.zeros(n_samples)
    dHdt = jax.vmap(jnp.dot, [0, 0])(qd, tau_pred)
    return tau_pred, H, dHdt

def loss_fn(params, q, qd, qdd, tau, model, n_dof, norm_tau):
    tau_pred, H_pred, dHdt_pred = dynamics_model(params, None, q, qd, qdd, model)

    # Inverse Error:
    tau_error = jnp.sum((tau - tau_pred)**2 / norm_tau, axis=-1)
    mean_inverse_error = jnp.mean(tau_error)
    var_inverse_error = jnp.mean(tau_error)

    # Temporal Energy Conservation:
    dHdt = jax.vmap(jnp.dot, [0, 0])(qd, tau)
    dHdt_error = (dHdt_pred - dHdt) ** 2
    mean_energy_error = jnp.mean(dHdt_error)
    var_energy_error = jnp.var(dHdt_error)

    # Compute Loss
    loss = mean_inverse_error

    logs = {
        'n_batch': 1,
        'loss': loss,
        'inverse_mean': mean_inverse_error,
        'inverse_var': var_inverse_error,
        'energy_mean': mean_energy_error,
        'energy_var': var_energy_error,
    }
    return loss, logs

class ReplayMemory(object):
    def __init__(self, maximum_number_of_samples, minibatch_size, dim):

        # General Parameters:
        self._max_samples = maximum_number_of_samples
        self._minibatch_size = minibatch_size
        self._dim = dim
        self._data_idx = 0
        self._data_n = 0

        # Sampling:
        self._sampler_idx = 0
        self._order = None

        # Data Structure:
        self._data = []
        for i in range(len(dim)):
            self._data.append(np.empty((self._max_samples, ) + dim[i]))

    def __iter__(self):
        # Shuffle data and reset counter:
        self._order = np.random.permutation(self._data_n)
        self._sampler_idx = 0
        return self

    def __next__(self):
        if self._order is None or self._sampler_idx >= self._order.size:
            raise StopIteration()

        tmp = self._sampler_idx
        self._sampler_idx += self._minibatch_size
        self._sampler_idx = min(self._sampler_idx, self._order.size)

        batch_idx = self._order[tmp:self._sampler_idx]

        # Reject Batches that have less samples:
        if batch_idx.size < self._minibatch_size:
            raise StopIteration()

        out = [x[batch_idx] for x in self._data]
        return out

    def add_samples(self, data):
        assert len(data) == len(self._data)

        # Add samples:
        add_idx = self._data_idx + np.arange(data[0].shape[0])
        add_idx = np.mod(add_idx, self._max_samples)

        for i in range(len(data)):
            self._data[i][add_idx] = data[i][:]

        # Update index:
        self._data_idx = np.mod(add_idx[-1] + 1, self._max_samples)
        self._data_n = min(self._data_n + data[0].shape[0], self._max_samples)

        # Clear excessive GPU Memory:
        del data

    def shuffle(self):
        self._order = np.random.permutation(self._data_idx)
        self._sampler_idx = 0

    def get_full_mem(self):
        out = [x[:self._data_n] for x in self._data]
        return out

    def not_empty(self):
        return self._data_n > 0
    
def load_dataset(n_characters=1, filename="combined_data.pickle", test_label=("3")):

    with open(filename, 'rb') as f:
        data = pickle.load(f)

    # data["labels"] = ["1","2","3"]
    n_dof = 3

    # Random Test Set:
    test_idx = np.random.choice(len(data["labels"]), n_characters, replace=False)

    # Specified Test Set:
    # test_idx = [data["labels"].index(x) for x in test_label]

    dt = np.concatenate([data["t"][idx][1:] - data["t"][idx][:-1] for idx in test_idx])
    dt_mean, dt_var = np.mean(dt), np.var(dt)
    # assert dt_var < 1.e-12

    train_labels, test_labels = [], []
    train_qp, train_qv, train_qa, train_tau = np.zeros((0, n_dof)), np.zeros((0, n_dof)), np.zeros((0, n_dof)), np.zeros((0, n_dof))
    test_qp, test_qv, test_qa, test_tau = np.zeros((0, n_dof)), np.zeros((0, n_dof)), np.zeros((0, n_dof)), np.zeros((0, n_dof))
    test_m, test_c, test_g = np.zeros((0, n_dof)), np.zeros((0, n_dof)), np.zeros((0, n_dof))

    divider = [0, ]   # Contains idx between characters for plotting

    for i in range(len(data["labels"])):

        if i in test_idx:
            test_labels.append(data["labels"][i])
            test_qp = np.vstack((test_qp, data["qp"][i]))
            test_qv = np.vstack((test_qv, data["qv"][i]))
            test_qa = np.vstack((test_qa, data["qa"][i]))
            test_tau = np.vstack((test_tau, data["tau"][i]))

            if tau_decomposition:
                test_m = np.vstack((test_m, data["m"][i]))
                test_c = np.vstack((test_c, data["c"][i]))
                test_g = np.vstack((test_g, data["g"][i]))
            else:
                test_m = np.vstack((test_m, np.zeros_like(data["qp"][i])))
                test_c = np.vstack((test_c, np.zeros_like(data["qp"][i])))
                test_g = np.vstack((test_g, np.zeros_like(data["qp"][i])))

            divider.append(test_qp.shape[0])

        else:
            train_labels.append(data["labels"][i])
            train_qp = np.vstack((train_qp, data["qp"][i]))
            train_qv = np.vstack((train_qv, data["qv"][i]))
            train_qa = np.vstack((train_qa, data["qa"][i]))
            train_tau = np.vstack((train_tau, data["tau"][i]))

    return (train_labels, train_qp, train_qv, train_qa, train_tau), \
           (test_labels, test_qp, test_qv, test_qa, test_tau, test_m, test_c, test_g),\
           divider, dt_mean

def compute_error_dict(delan_tau, test_tau, test_labels, divider):
    """
    Creates a dictionary:
        key   -> test label
        value -> error array (num_samples_i, n_dof)

    Args:
        pred: predicted values (N, n_dof)
        gt: ground truth values (N, n_dof)
        labels: list of test labels
        divider: list of indices marking boundaries

    Returns:
        dict[label] = error array
    """
    error_dict = {}

    for i, label in enumerate(test_labels):
        start = divider[i]
        end = divider[i + 1]

        # Slice per character
        delan_tau_i = delan_tau[start:end]
        test_tau_i = test_tau[start:end]

        # Compute squared error (you can change this if needed)
        error_i = (delan_tau_i - test_tau_i) ** 2

        error_dict[label] = error_i

    return error_dict

def add_errors_to_csv(error_dict, folder_path, prefix="error"):
    """
    Adds error columns (error1, error2, error3, ...) to each CSV file using csv module.

    Args:
        error_dict: dict[label] = (num_samples, n_dof) error array
        folder_path: directory containing CSV files (one per label)
        prefix: column name prefix (default: "error")

    Assumes:
        CSV filename = <label>.csv
    """

    for label, error_array in error_dict.items():
        file_path = os.path.join(folder_path, f"path_0{80+int(label)}_joint_states_1.csv")

        if not os.path.exists(file_path):
            print(f"[WARNING] File not found for label {label}: {file_path}")
            continue

        # Read existing CSV
        with open(file_path, "r", newline="") as f:
            reader = list(csv.reader(f))

        header = reader[0]
        rows = reader[1:]

        # Check length match
        if len(rows) != error_array.shape[0]:
            print(f"[ERROR] Length mismatch for {label}: CSV={len(rows)}, error={error_array.shape[0]}")
            continue

        # Add new column names
        n_dof = error_array.shape[1]
        error_headers = [f"{prefix}{i+1}" for i in range(n_dof)]
        new_header = header + error_headers

        # Append error values row-wise
        new_rows = []
        for i, row in enumerate(rows):
            error_values = error_array[i]
            # Convert to string for CSV writing
            error_values_str = [str(val) for val in error_values]
            new_rows.append(row + error_values_str)

        new_file_path = os.path.join(folder_path, f"{label}.csv")
        # Write back (overwrite file)
        with open(new_file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(new_header)
            writer.writerows(new_rows)

        print(f"[INFO] Updated {new_file_path}")

activations = {
    'tanh': jnp.tanh,
    'softplus': jax.nn.softplus,
}

seed = 0
cuda = 0
render = 1
load_model = 1
save_model = 0
rng_key = jax.random.PRNGKey(seed)
tau_decomposition = 0

# Construct Hyperparameters:
hyper = {
    'n_width': 64,
    'n_depth': 2,
    'n_minibatch': 300,
    'diagonal_epsilon': 0.1,
    'diagonal_shift': 2.0,
    'activation': 'tanh',
    'learning_rate': 1.e-04,
    'weight_decay': 1.e-5,
    'max_epoch': int(5 * 1e3),
    'black_box_model_type': dynamics_network,
    }

if load_model:
    with open(f"./data/fyp_jax_blackbox_50.jax", 'rb') as f:
        data = pickle.load(f)

    hyper = data["hyper"]
    params = data["params"]

else:
    params = None

# Read the dataset:
train_data, test_data, divider, dt = load_dataset(
                                    n_characters= 3,
                                    filename="./data/100_trajectory_torques_rows/combined_data.pickle",
                                    test_label=["101","102"])

train_labels, train_qp, train_qv, train_qa, train_tau = train_data
test_labels, test_qp, test_qv, test_qa, test_tau, test_m, test_c, test_g = test_data
n_dof = test_qp.shape[-1]

# Generate Replay Memory:
mem_dim = ((n_dof,), (n_dof,), (n_dof,), (n_dof,))
mem = ReplayMemory(train_qp.shape[0], hyper["n_minibatch"], mem_dim)
mem.add_samples([train_qp, train_qv, train_qa, train_tau])

print("\n\n################################################")
print("Characters:")
print("   Test Characters = {0}".format(test_labels))
print("  Train Characters = {0}".format(train_labels))
print("# Training Samples = {0:05d}".format(int(train_qp.shape[0])))
print("")

# Training Parameters:
print("\n################################################")
print("Training Black Box Network:\n")

# Construct DeLaN:
t0 = time.perf_counter()

black_box_model_fn = hk.transform(functools.partial(
    dynamics_network,
    n_dof=n_dof,
    shape=(hyper['n_width'],) * hyper['n_depth'],
    activation=activations[hyper['activation']]
))

q, qd, qdd, tau = [jnp.array(x) for x in next(iter(mem))]
rng_key, init_key = jax.random.split(rng_key)

 # Initialize Parameters:
if params is None:
    params = black_box_model_fn.init(init_key, q[0:1], qd[0:1], qdd[0:1])

# Trace Model:
black_box_model = black_box_model_fn.apply
black_box_dynamics_model = jax.jit(functools.partial(dynamics_model, black_box_model=black_box_model))
_ = black_box_dynamics_model(params, None, q[:1], qd[:1], qdd[:1])
t_build = time.perf_counter() - t0
print(f"Black Box Model Build Time     = {t_build:.2f}s")

# Generate & Initialize the Optimizer:
t0 = time.perf_counter()

optimizer = optax.adamw(
    learning_rate=hyper['learning_rate'],
    weight_decay=hyper['weight_decay']
)

opt_state = optimizer.init(params)

loss_fn = functools.partial(
        loss_fn,
        model=black_box_model,
        n_dof=n_dof,
        norm_tau=jnp.var(train_tau, axis=0)
    )

def update_fn(params, opt_state, q, qd, qdd, tau, logs):
    (_, batch_logs), grads = jax.value_and_grad(loss_fn, 0, has_aux=True)(params, q, qd, qdd, tau)

    updates, opt_state = optimizer.update(grads, opt_state, params)
    params = optax.apply_updates(params, updates)

    logs = jax.tree.map(lambda x, y: x + y, logs, batch_logs)
    return params, opt_state, logs

logs = {
    'n_batch': 1,
    'loss': jnp.array(0.0),
    'inverse_mean': jnp.array(0.0), 'inverse_var': jnp.array(0.0),
    'energy_mean': jnp.array(0.0), 'energy_var': jnp.array(0.0),
}

update_fn = jax.jit(update_fn)
_ = update_fn(params, opt_state, q[:1], qd[:1], qdd[:1], tau[:1], logs)

t_build = time.perf_counter() - t0
print(f"Optimizer Build Time = {t_build:.2f}s")

# Start Training Loop:
t0_start = time.perf_counter()

print("")
epoch_i = 0
while epoch_i < hyper['max_epoch'] and not load_model:
    logs = jax.tree.map(lambda x: x * 0.0, logs)

    for data_batch in mem:
        t0_batch = time.perf_counter()

        q, qd, qdd, tau = [jnp.array(x) for x in data_batch]
        params, opt_state, logs = update_fn(params, opt_state, q, qd, qdd, tau, logs)
        t_batch = time.perf_counter() - t0_batch

    # Update Epoch Loss & Computation Time:
    epoch_i += 1
    logs = jax.tree.map(lambda x: x/logs['n_batch'], logs)

    if epoch_i == 1 or np.mod(epoch_i, 100) == 0:
        print("Epoch {0:05d}: ".format(epoch_i), end=" ")
        print(f"Time = {time.perf_counter() - t0_start:05.1f}s", end=", ")
        print(f"Loss = {logs['loss']:.1e}", end=", ")
        print(f"Inv = {logs['inverse_mean']:.1e} \u00B1 {1.96 * np.sqrt(logs['inverse_var']):.1e}", end=", ")
        print(f"Power = {logs['energy_mean']:.1e} \u00B1 {1.96 * np.sqrt(logs['energy_var']):.1e}")

# Save the Model:
if save_model:
    with open(f"./data/fyp_jax_blackbox_50.jax", "wb") as file:
        pickle.dump(
            {"epoch": epoch_i,
            "hyper": hyper,
            "params": params,
            "seed": seed},
            file)
        
print("\n################################################")
print("Evaluating Black Box Dynamics Model:")

# Convert NumPy samples to torch:
q, qd, qdd = jnp.array(test_qp), jnp.array(test_qv), jnp.array(test_qa)
zeros = jnp.zeros_like(q)

# Compute the torque decomposition:
delan_g = black_box_dynamics_model(params, None, q, zeros, zeros)[0]
delan_c = black_box_dynamics_model(params, None, q, qd, zeros)[0] - delan_g
delan_m = black_box_dynamics_model(params, None, q, zeros, qdd)[0] - delan_g

t0_evaluation = time.perf_counter()
delan_tau = black_box_dynamics_model(params, None, q, qd, qdd)[0]
t_eval = (time.perf_counter() - t0_evaluation) / float(q.shape[0])

# Compute Errors:
test_dEdt = np.sum(test_tau * test_qv, axis=1).reshape((-1, 1))
err_g = 1. / float(test_qp.shape[0]) * np.sum((delan_g - test_g) ** 2)
err_m = 1. / float(test_qp.shape[0]) * np.sum((delan_m - test_m) ** 2)
err_c = 1. / float(test_qp.shape[0]) * np.sum((delan_c - test_c) ** 2)
err_tau = 1. / float(test_qp.shape[0]) * np.sum((delan_tau - test_tau) ** 2)

# error_dict = compute_error_dict(delan_tau, test_tau, test_labels, divider)
# add_errors_to_csv(error_dict, "./data/100_trajectory_torques_rows", prefix="error")

print("\nPerformance:")
print("                Torque MSE = {0:.3e}".format(err_tau))
print("              Inertial MSE = {0:.3e}".format(err_m))
print("Coriolis & Centrifugal MSE = {0:.3e}".format(err_c))
print("         Gravitational MSE = {0:.3e}".format(err_g))
print("      Comp Time per Sample = {0:.3e}s / {1:.1f}Hz".format(t_eval, 1./t_eval))

print("\n################################################")
print("Plotting Performance:")

def get_limits(gt, pred, low_clip=-0.01, high_clip=0.01, scale_low=1.2, scale_high=1.5):
    low = np.clip(scale_low * np.min(np.vstack((gt, pred)), axis=0), -np.inf, low_clip)
    high = np.clip(scale_high * np.max(np.vstack((gt, pred)), axis=0), high_clip, np.inf)
    return low, high

num_joints = test_tau.shape[1]  # dynamic number of joints

y_t_low, y_t_max = get_limits(test_tau, delan_tau)
y_m_low, y_m_max = get_limits(test_m, delan_m, scale_high=1.2)
y_c_low, y_c_max = get_limits(test_c, delan_c, scale_high=1.2)
y_g_low, y_g_max = get_limits(test_g, delan_g, scale_high=1.2)

plot_alpha = 0.8
color_bb = "r"    # black-box network color
color_gt = "k"    # ground truth color

ticks = (np.array(divider[:-1]) + np.array(divider[1:])) / 2

# Figure with N rows × 4 columns
fig = plt.figure(figsize=(24.0/1.54, (4 * num_joints)/1.54), dpi=100)
fig.subplots_adjust(left=0.08, bottom=0.08, right=0.98, top=0.95,
                    wspace=0.3, hspace=0.25)
fig.canvas.manager.set_window_title(f"Seed = {seed}")

# Column titles
titles = ["tau", "H(q)*qdd", "c(q, qd)", "g(q)"]

# Data blocks in column order
gt_blocks = [test_tau,  test_m,  test_c,  test_g] # Ground truth
nn_blocks = [delan_tau, delan_m, delan_c, delan_g] # Neural Network
lim_low   = [y_t_low,   y_m_low, y_c_low, y_g_low]
lim_high  = [y_t_max,   y_m_max, y_c_max, y_g_max]

# Plotting Loop
for j in range(num_joints):
    for col in range(4):
        ax = fig.add_subplot(num_joints, 4, j*4 + col + 1)

        # Add joint name on the left side of the row
        if col == 0:
            ax.text(
                s = f"Joint {j}", x=-0.35, y=0.5,
                fontsize=12, fontweight="bold",
                rotation=90, ha="center", va="center",
                transform=ax.transAxes
            )

        # Column title only for top row
        if j == 0:
            ax.set_title(titles[col], fontsize=12)

        # Plot limits
        ax.set_ylabel("Torque [Nm]")
        ax.set_ylim(lim_low[col][j], lim_high[col][j])
        ax.set_xticks(ticks)
        ax.set_xticklabels(test_labels)
        ax.set_xlim(divider[0], divider[-1])
        ax.vlines(
            divider,
            lim_low[col][j],
            lim_high[col][j],
            linestyles="--",
            lw=0.5,
            alpha=1.0
        )

        # Plot ground truth + DeLaN
        ax.plot(gt_blocks[col][:, j], color=color_gt)
        ax.plot(nn_blocks[col][:, j], color=color_bb, alpha=plot_alpha)

        # Add legend only on the first subplot
        if j == 0 and col == 0:
            legend = [
                mp.patches.Patch(color=color_bb, label="Black-Box Network"),
                mp.patches.Patch(color="k", label="Ground Truth")
            ]
            ax.legend(handles=legend, bbox_to_anchor=(0.0, 1.0),
                      loc="upper left", framealpha=1.0)

if render:
    plt.show()

print("\n################################################\n\n\n")
