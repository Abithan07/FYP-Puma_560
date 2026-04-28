#!/usr/bin/env python3
"""DNN predictor wrapper (DeLaN + GRU) — extracted from torque_publisher_dnn
Exposes DNNInferenceEngine(delan_path, gru_path, scaler_path).predict(q,qd,qdd)
"""
import os
import numpy as np
from collections import deque

class DNNInferenceEngine:
    SEQ_LEN = 128
    N_DOF = 3
    INPUT_DIM = 12
    HIDDEN_DIM = 64
    N_LAYERS = 4

    def __init__(self, delan_path, gru_path, scaler_path):
        # Delay heavy imports until init
        os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
        os.environ['XLA_PYTHON_CLIENT_ALLOCATOR']   = 'platform'
        import torch
        import dill as pickle
        import joblib
        import jax
        import jax.numpy as jnp
        import haiku as hk
        import functools

        self._torch = torch
        self._jnp = jnp
        self._jax = jax
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Load DeLaN
        with open(delan_path, 'rb') as f:
            saved = pickle.load(f)
        params, hyper = saved['params'], saved['hyper']
        activations = {'tanh': jnp.tanh, 'softplus': jax.nn.softplus}

        def mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift):
            n_out = int((n_dof**2 + n_dof) / 2)
            idx_diag = (np.arange(n_dof) + 1)
            idx_diag = (idx_diag * (idx_diag + 1) / 2 - 1).astype(int)
            idx_tril = np.setdiff1d(np.arange(n_out), idx_diag)
            cat_idx = np.hstack((idx_diag, idx_tril))
            idx = np.arange(cat_idx.size)[np.argsort(cat_idx)]
            mat_idx = np.tril_indices(n_dof)
            net = hk.nets.MLP(output_sizes=shape + (n_out,), activation=activation,
                               name='mass_matrix')
            z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
            l_d, l_od = jnp.split(net(z), [n_dof], axis=-1)
            l_d = jax.nn.softplus(l_d + shift) + epsilon
            vec = jnp.concatenate((l_d, l_od), axis=-1)[..., idx]
            L = jnp.zeros((n_dof, n_dof)).at[mat_idx].set(vec[:])
            return jnp.matmul(L, L.transpose())

        def kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift):
            M = mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift)
            return 0.5 * jnp.dot(qd, jnp.dot(M, qd))

        def potential_energy_fn(q, shape, activation):
            net = hk.nets.MLP(output_sizes=shape + (1,), activation=activation,
                               name='potential_energy')
            z = jnp.concatenate([jnp.cos(q), jnp.sin(q)], axis=-1)
            return net(z)

        def structured_lagrangian_fn(q, qd, n_dof, shape, activation, epsilon, shift):
            return (kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift)
                    - potential_energy_fn(q, shape, activation).squeeze())

        def dynamics_model(params, key, q, qd, qdd, tau, lagrangian, n_dof):
            vmap_dim = (None, None, 0, 0)
            batch_mm = jax.vmap(jnp.matmul, (0, 0))
            lag_vg = jax.value_and_grad(lagrangian, argnums=[2, 3])
            L, (dLdq, dLdqd) = jax.vmap(lag_vg, vmap_dim)(params, key, q, qd)
            lag_hess = jax.hessian(lagrangian, argnums=[2, 3])
            (_, (d2L_dqddq, d2Ld2qd)) = jax.vmap(lag_hess, vmap_dim)(params, key, q, qd)
            tau_pred = batch_mm(d2Ld2qd, qdd) + batch_mm(d2L_dqddq, qd) - dLdq
            return None, tau_pred, None, None

        shape = (hyper['n_width'],) * hyper['n_depth']
        activation = activations[hyper['activation']]
        epsilon = hyper['diagonal_epsilon']
        shift = hyper['diagonal_shift']

        lag_fn = hk.transform(functools.partial(
            structured_lagrangian_fn, n_dof=self.N_DOF,
            shape=shape, activation=activation, epsilon=epsilon, shift=shift))

        self._delan_params = params
        import functools as _functools
        self._delan_fn = jax.jit(_functools.partial(dynamics_model, lagrangian=lag_fn.apply, n_dof=self.N_DOF))

        # warm up
        _q0 = jnp.zeros((1, self.N_DOF))
        _qd0 = jnp.zeros((1, self.N_DOF))
        try:
            self._delan_fn(params, None, _q0, _qd0, _q0, _q0)
        except Exception:
            pass

        # GRU residual
        import torch.nn as nn
        class GRUResidual(nn.Module):
            def __init__(self):
                super().__init__()
                self.gru = nn.GRU(input_size=DNNInferenceEngine.INPUT_DIM,
                                  hidden_size=DNNInferenceEngine.HIDDEN_DIM,
                                  num_layers=DNNInferenceEngine.N_LAYERS,
                                  batch_first=True, dropout=0.1)
                self.fc = nn.Linear(DNNInferenceEngine.HIDDEN_DIM, DNNInferenceEngine.N_DOF)
            def forward(self, x):
                h, _ = self.gru(x)
                return self.fc(h[:, -1, :])

        self._gru = GRUResidual().to(self.device)
        self._gru.load_state_dict(torch.load(gru_path, map_location=self.device))
        self._gru.eval()

        # scaler
        self._scaler = joblib.load(scaler_path)

        self._buffer = deque(maxlen=self.SEQ_LEN)

    def predict(self, q, qd, qdd):
        jnp = self._jnp
        q_j = jnp.array(q, dtype=jnp.float32)[None, :]
        qd_j = jnp.array(qd, dtype=jnp.float32)[None, :]
        qdd_j = jnp.array(qdd, dtype=jnp.float32)[None, :]

        _, tau_d, _, _ = self._delan_fn(self._delan_params, None, q_j, qd_j, qdd_j, jnp.zeros_like(q_j))
        tau_delan = np.array(tau_d[0], dtype=np.float64)

        raw_feat = np.concatenate([q, qd, qdd, tau_delan]).reshape(1, -1)
        self._buffer.append(self._scaler.transform(raw_feat)[0])

        if len(self._buffer) < self.SEQ_LEN:
            return tau_delan.copy(), tau_delan.copy(), False

        window = self._torch.FloatTensor(np.array(self._buffer)[None, :, :]).to(self.device)
        with self._torch.no_grad():
            tau_res = self._gru(window).cpu().numpy()[0]
        tau_dnn = tau_delan + tau_res
        return tau_dnn, tau_delan, True
