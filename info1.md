DNN-Based Novel Computed Torque Controller for a 3-DOF Modified PUMA 560
1. Project Overview

This project focuses on the design and implementation of a Deep Neural Network (DNN)-based computed torque controller for a robotic manipulator using a Deep Lagrangian Network (DeLaN).

The system is built around a 3-DOF modified PUMA 560 robotic arm, where analytical dynamics are first used to generate a dataset, and later replaced by a learned dynamics model.

2. System Workflow

The complete workflow consists of the following stages:

- Trajectory Generation
- Inverse Dynamics Computation
- Physics-Based Validation in Gazebo
- Dataset Preparation
- Learning Dynamics via DeLaN
- Controller Design and Evaluation
3. Trajectory Generation (Python)

Smooth S-curve trajectories are generated in joint space to ensure continuity in:

- Position: q(t)
- Velocity: q̇(t)
- Acceleration: q̈(t)

These trajectories are sampled at a frequency of 100 Hz.

Python Code: Trajectory Generation
```python
import argparse
import numpy as np
import pandas as pd
import os

# ---------------- USER INPUT ----------------
dt = 0.01
possible_T = np.arange(12, 25, 4)
v_max = 2
a_max = 7

parser = argparse.ArgumentParser()
parser.add_argument("path_id", type=int, help="Trajectory file ID, e.g. 601")
parser.add_argument("--seed", type=int, default=None, help="Random seed (optional)")
args = parser.parse_args()

path_id = args.path_id

if args.seed is not None:
    np.random.seed(args.seed)

# ---------------- DIRECTORIES ----------------
base_dir = "/home/priyankan/Desktop/FYP-Puma_560/DNN_test/Data"
summary_dir = "/home/priyankan/Desktop/FYP-Puma_560/DNN_test"

angle_dir = os.path.join(base_dir, "Angles")
xyz_dir   = os.path.join(base_dir, "XYZ")
traj_dir  = os.path.join(base_dir, "Trajectory")

summary_file = os.path.join(summary_dir, "summary.csv")
summary_deg_file = os.path.join(summary_dir, "summary_deg.csv")

for d in [angle_dir, xyz_dir, traj_dir]:
    os.makedirs(d, exist_ok=True)

# ---------------- LOAD SUMMARY ----------------
if os.path.exists(summary_file):
    summary = pd.read_csv(summary_file, header=None, dtype=np.float64).values
else:
    summary = np.empty((0,4))

# ---------------- DH PARAMETERS ----------------
alpha = np.array([0, -np.pi/2, 0])
a_dh  = np.array([0, 0, 0.4318])
d_dh  = np.array([0, 0.2435, -0.0934])

# Fast NumPy DH
def dh_numpy(alpha, a, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)

    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa, ca, ca*d],
        [0, 0, 0, 1]
    ])

# ---------------- TRAJECTORY GENERATION ----------------
q_start = np.radians([0, 45, 135])
q_min = np.radians([-80, 25, 100])
q_max = np.radians([80, 45, 170])

valid = False
max_attempts = 1000
attempts = 0

while not valid and attempts < max_attempts:
    attempts += 1

    q_end = q_min + (q_max - q_min) * np.random.rand(3)
    delta_q = np.abs(q_end - q_start)

    T_vel = 1.875 * np.max(delta_q / v_max)
    T_acc = np.max(np.sqrt(5.77 * delta_q / a_max))
    T_min = max(T_vel, T_acc)

    T_rand = np.random.choice(possible_T)
    T_total = max(T_min, T_rand)

    candidate = np.append(q_end, T_total)

    if summary.shape[0] == 0 or not np.any(np.all(np.abs(summary - candidate) < 1e-4, axis=1)):
        valid = True

if not valid:
    raise RuntimeError("Could not generate a unique trajectory.")

# ---------------- TIME ----------------
t = np.arange(0, T_total + dt, dt)
tau = t / T_total

# ---------------- MINIMUM JERK ----------------
f   = 10*tau**3 - 15*tau**4 + 6*tau**5
fd  = (30*tau**2 - 60*tau**3 + 30*tau**4) / T_total
fdd = (60*tau - 180*tau**2 + 120*tau**3) / T_total**2

# ---------------- TRAJECTORY ----------------
q_traj = np.zeros((len(t), 3))
dq_traj = np.zeros_like(q_traj)
ddq_traj = np.zeros_like(q_traj)

for j in range(3):
    dqj = q_end[j] - q_start[j]
    q_traj[:, j] = q_start[j] + dqj * f
    dq_traj[:, j] = dqj * fd
    ddq_traj[:, j] = dqj * fdd

# ---------------- FORWARD KINEMATICS ----------------
xyz = np.zeros_like(q_traj)

for k in range(len(t)):
    T1 = dh_numpy(alpha[0], a_dh[0], d_dh[0], q_traj[k,0])
    T2 = dh_numpy(alpha[1], a_dh[1], d_dh[1], q_traj[k,1])
    T3 = dh_numpy(alpha[2], a_dh[2], d_dh[2], q_traj[k,2])

    T = T1 @ T2 @ T3
    xyz[k, :] = T[:3, 3]

# ---------------- SAVE FILES ----------------
np.savetxt(os.path.join(angle_dir, f'path_{path_id:03d}_angles.csv'), q_traj, delimiter=',')
np.savetxt(os.path.join(xyz_dir, f'path_{path_id:03d}_xyz.csv'), xyz, delimiter=',')

# Trajectory CSV (cleaner using pandas)
traj_matrix = np.hstack([t.reshape(-1,1), q_traj, dq_traj, ddq_traj])

row_titles = ['t','dp1','dp2','dp3','dv1','dv2','dv3','da1','da2','da3']
df = pd.DataFrame(traj_matrix.T)
df.insert(0, 'var', row_titles)

traj_file = os.path.join(traj_dir, f'path_{path_id:03d}_trajectories.csv')
df.to_csv(traj_file, index=False, header=False)

# ---------------- UPDATE SUMMARY ----------------
summary = np.vstack([summary, candidate])
pd.DataFrame(summary).to_csv(summary_file, index=False, header=False)

# Keep a derived file where columns 1-3 are converted to degrees.
# This is refreshed every time summary.csv is updated.
summary_deg = summary.copy()
summary_deg[:, :3] = np.degrees(summary_deg[:, :3])
pd.DataFrame(summary_deg).to_csv(summary_deg_file, index=False, header=False)

print(f"Trajectory {path_id} generated and saved.")
print(f"Start (deg): {np.degrees(q_start)}")
print(f"End   (deg): {np.degrees(q_end)}")
print(f"T_total (s): {T_total:.4f}")
```
4. Inverse Dynamics Computation (MATLAB)

A manually derived rigid-body dynamic model of the 3-DOF modified PUMA 560 is implemented in MATLAB.

The model includes:

- Link masses
- Inertia tensors
- Kinematic parameters

Given trajectory inputs:

- q(t), q̇(t), q̈(t)

The inverse dynamics computes:

- Joint torques τ(t)

Note: Friction and external disturbances are not included, resulting in an idealized dataset.

MATLAB Code: Dynamic Model and Torque Computation
```matlab
clc; clear; close all;

% rowNames = { ...
%     't', ...
%     'dp1','dp2','dp3', ...
%     'dv1','dv2','dv3', ...
%     'da1','da2','da3', ...
%     'tau1','tau2','tau3', ...
%     'm1','m2','m3', ...
%     'c1','c2','c3', ...
%     'g1','g2','g3' ...
%     };
%% 1. Symbolic Declarations
syms q1 q2 q3
q = [q1; q2; q3];
g = 9.81;

% Link parameters
a1=0;   a2=-pi/2;     a3=0;      % link twist alpha_{i-1}
L1=0;   L2=0;         L3=0.4318;  % link length l_{i-1}
d1=0;   d2=0.2435;     d3=-0.0934; % Joint offset d_i

% Link CoG
CoG1 = [0; 0; 0];   % Link-1 CoG at joint axis
CoG2 = [0.068; 0.006; -0.016];
CoG3 = [0.000; -0.143; 0.014];

% 5. Base point as {1} is located at (0,0,0)
pb=[0;0;-0.6718]; xb=pb(1); yb=pb(2); zb=pb(3);

% 6. Link masses
m1=0.01; m2=17.4; m3=4.8; % m1 is not required because link
% 1 only rotates around its z axis. Its inertia is seperately given

% 7. Link and motor inertia from the Stanford paper
Im1=1.14; Im2=4.71; Im3=0.83;
I1xx=0.745; I1yy=0.745; I1zz=0.35+Im1;
I2xx=2.6245; I2yy=2.6245; I2zz=0.539+Im2;
I3xx=0.458; I3yy=0.458; I3zz=0.086+Im3;

% 7.1 Inertia matrices
I1=[I1xx 0 0; 0 I1yy 0; 0 0 I1zz];
I2=[I2xx 0 0; 0 I2yy 0; 0 0 I2zz];
I3=[I3xx 0 0; 0 I3yy 0; 0 0 I3zz];

%% 2. Homogeneous Transform Function
T = @(alpha,len,ofs,theta) [ ...
    cos(theta) -sin(theta) 0 len;
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*ofs;
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*ofs;
    0 0 0 1];

% HT matrices
T01 = T(a1,L1,d1,q1);
T12 = T(a2,L2,d2,q2);
T23 = T(a3,L3,d3,q3);
T02 = T01*T12;
T03 = T02*T23;

R01 = T01(1:3,1:3); 
R02 = T02(1:3,1:3); 
R03 = T03(1:3,1:3);

%% 3. Jacobians
Jw1=[R01(:,3) [0;0;0] [0;0;0]];
Jw2=[R01(:,3) R02(:,3) [0;0;0]];
Jw3=[R01(:,3) R02(:,3) R03(:,3)];

% rc1=T01(1:3,4);
rc1=T01(1:3,4)+R01*CoG1;
rc2=T02(1:3,4)+R02*CoG2;
rc3=T03(1:3,4)+R03*CoG3;

v1q1=diff(rc1,q1); v1q2=diff(rc1,q2); v1q3=diff(rc1,q3);
v2q1=diff(rc2,q1); v2q2=diff(rc2,q2); v2q3=diff(rc2,q3);
v3q1=diff(rc3,q1); v3q2=diff(rc3,q2); v3q3=diff(rc3,q3);

Jv1=[v1q1 v1q2 v1q3]; 
Jv2=[v2q1 v2q2 v2q3]; 
Jv3=[v3q1 v3q2 v3q3];

%% 4. Inertia matrix D
D = m1*(Jv1.')*Jv1 + Jw1.'*R01*I1*R01.'*Jw1 + ...
    m2*(Jv2.')*Jv2 + Jw2.'*R02*I2*R02.'*Jw2 + ...
    m3*(Jv3).'*Jv3 + Jw3.'*R03*I3*R03.'*Jw3;

%% 5. Gravity vector
P = g*( m1*rc1(3) + m2*rc2(3) + m3*rc3(3));
% P = g*( m1*rc1(2) + m2*rc2(2) + m3*rc3(2));
G = [diff(P,q1); diff(P,q2); diff(P,q3)];

%% 6. Christoffel symbols C
C = sym(zeros(3,3,3));
for i=1:3
    for j=1:3
        for k=1:3
            C(i,j,k) = 0.5*(diff(D(i,j),q(k)) + diff(D(i,k),q(j)) - diff(D(j,k),q(i)));
        end
    end
end

%% 7. Convert symbolic to numeric functions
D_func = matlabFunction(D,'Vars',{q1,q2,q3});
G_func = matlabFunction(G,'Vars',{q1,q2,q3});
C_func = matlabFunction(C,'Vars',{q1,q2,q3});

%% 8. Trajectory File
% cd('C:\Users\ROG\Desktop\Trajectory Gen\Datasets\Paths_1\Trajectories');
cd('C:\Users\Priyankan\Desktop\data');
fileName = 'move_q3_traj.csv'; 

%% 9. Parallel computation of torques (row-wise output)
% parfor f = 1:length(files)
data = readmatrix(fullfile(fileName));
N = size(data,1);

% If you already have time in your CSV, replace this with:-------------
t  = data(:,1);
dp = data(:,2:4); dv = data(:,5:7); da = data(:,8:10);
% Ts = 0.01;              % sampling time [s]
% t  = (0:N-1)' * Ts;     % time vector: 0, 0.01, 0.02, ...
% dp = data(:,1:3);       % positions q1..q3
% dv = data(:,4:6);       % velocities dq1..dq3
% da = data(:,7:9);       % accelerations ddq1..ddq3

tau_out = zeros(N,3);
M_out   = zeros(N,3);   % M(q)*ddq
C_out   = zeros(N,3);   % Coriolis/centripetal vector
G_out   = zeros(N,3);   % Gravity vector

% local copies inside parfor
Df = D_func; 
Gf = G_func; 
Cf = C_func;

for k = 1:N
    qk   = dp(k,:).';   % [q1;q2;q3]
    dqk  = dv(k,:).';   % [dq1;dq2;dq3]
    ddqk = da(k,:).';   % [ddq1;ddq2;ddq3]

    Dk = Df(qk(1),qk(2),qk(3));   % 3x3
    Gk = Gf(qk(1),qk(2),qk(3));   % 3x1
    Ck = Cf(qk(1),qk(2),qk(3));   % 3x3x3

    % Compute Coriolis/Centripetal vector C(q,dq)
    Coriolis = zeros(3,1);
    for i=1:3
        for j=1:3
            for k2=1:3
                Coriolis(i) = Coriolis(i) + Ck(i,j,k2)*dqk(j)*dqk(k2);
            end
        end
    end

    Mvec = Dk*ddqk;  % M(q)*ddq

    tau_out(k,:) = (Mvec + Coriolis + Gk).';

    M_out(k,:) = Mvec.';
    C_out(k,:) = Coriolis.';
    G_out(k,:) = Gk.';
end

outMatrix = [ ...
t.';             
dp.';            
dv.';            
da.';            
tau_out.';       
M_out.';         
C_out.';         
G_out.'          
];

% Row labels
rowNames = { ...
    't', ...
    'dp1','dp2','dp3', ...
    'dv1','dv2','dv3', ...
    'da1','da2','da3', ...
    'tau1','tau2','tau3', ...
    'm1','m2','m3', ...
    'c1','c2','c3', ...
    'g1','g2','g3' ...
    };

% Convert to cell array with row name in first column
outCell = [rowNames(:), num2cell(outMatrix)];

% Save file
% outputFile = fullfile(outputFolder, files(f).name);

[~, baseName, ext] = fileparts(fileName);
newName = sprintf('%s_joint_states%s', baseName, ext);

% Build full output path
outputFile = fullfile(newName);
writecell(outCell, outputFile);

fprintf('Processed %s\n', fileName);

% disp('All trajectories processed and row-wise data saved.');
```
5. Simulation in Gazebo (Validation Stage)

The generated trajectories and corresponding torques are validated in a Gazebo simulation environment.

Simulation Setup
- Robot: 3-DOF modified PUMA 560 (URDF-based)
- Control input: Joint torques (effort control)
- Command frequency: 100 Hz
- Environment: Ideal (no noise, no disturbances)

Objective

To verify that:

- Applying the computed torques results in accurate trajectory tracking.

Current Observation
- System exhibits instability
- Robot fails to track trajectories accurately

This suggests a mismatch between:

- Analytical model (MATLAB)
- Simulation model (Gazebo physics engine)
6. Dataset Preparation

Once the simulation is validated, a dataset is constructed:

Dataset Structure: (q, q̇, q̈) → τ

This dataset will be used to train a neural network model.

7. Learning Dynamics using DeLaN

A Deep Lagrangian Network (DeLaN) is trained to learn system dynamics.

Key Characteristics
- Input: q, q̇, q̈
- Output: Predicted torques τ̂
- Ensures energy-based consistency
- Preserves physical interpretability

The trained DeLaN model replaces the analytical inverse dynamics.

8. Controller Design

A feedforward torque controller is implemented using the trained DeLaN:

τ = τ_DeLaN(q, q̇, q̈)

For safety and robustness, a minimal feedback term (PD controller) may be added:

τ = τ_DeLaN + K_p(q_d - q) + K_d(q̇_d - q̇)
9. Evaluation Metrics

The system performance is evaluated using:

- Trajectory Tracking Error (Primary Metric)
- Stability of motion
10. Task Definition for the AI System
Objective

Develop a Gazebo simulation model of the 3-DOF modified PUMA 560 robotic arm that can accurately reproduce the behavior defined by the analytical dynamics.