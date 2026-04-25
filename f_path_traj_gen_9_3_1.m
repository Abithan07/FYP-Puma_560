%% ============================================================
%% PUMA-560 PATH + TRAJECTORY DATASET GENERATOR
%% Generates:
%% 1) Joint Angles
%% 2) End Effector XYZ
%% 3) Full trajectory (q,qd,qdd)
%% Constraints:
%% |velocity| ≤ 2 rad/s
%% |acceleration| ≤ 7 rad/s²
%% ------------------Joint limits of p560-----------------------
% q1_min = deg2rad(-160); q1_max = deg2rad(160);
% q2_min = -5*pi/4;       q2_max = pi/4;
% q3_min = -pi/4;         q3_max = 5*pi/4;
%% ============================================================

clearvars; close all; clc;

%% ===================== USER INPUT =====================

numPaths = input('Enter number of trajectories: ');

%% ===================== SETTINGS =====================

dt = 0.01;
possible_T = 12:4:24;

v_max = 2;
a_max = 7;

%% ===================== OUTPUT DIRECTORIES =====================

baseDir = 'C:\Users\Priyankan\Desktop\Trajectory Gen\Dataset 29-03';

angleDir = fullfile(baseDir,'Angles');
xyzDir   = fullfile(baseDir,'XYZ');
trajDir  = fullfile(baseDir,'Trajectories');

if ~exist(angleDir,'dir'); mkdir(angleDir); end
if ~exist(xyzDir,'dir'); mkdir(xyzDir); end
if ~exist(trajDir,'dir'); mkdir(trajDir); end

%% ===================== PATH RECORD =====================

recordFile = fullfile(baseDir,'path_records.csv');

if exist(recordFile,'file')
    pathRecord = readmatrix(recordFile);
else
    pathRecord = [];
end

%% ===================== PUMA DH PARAMETERS =====================

alpha = [0 -pi/2 0 pi/2];
a_dh  = [0 0 0.4318 0];
d_dh  = [0 0.2435 -0.0934 0.4331];

T_base = eye(4);
T_base(3,4) = 0.6718;

%% ===================== START CONFIGURATION =====================

q_start = deg2rad([0 45 135]);
q4 = 0;

%% ===================== END LIMITS =====================

q1_min = deg2rad(-100);
q1_max = deg2rad(100);

q2_min = deg2rad(-15);
q2_max = deg2rad(45);

q3_min = deg2rad(65);
q3_max = deg2rad(205);

%% ===================== VISUALIZATION =====================

figure('Color','w'); hold on; grid on; axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Generated PUMA-560 Paths')
view(3)

%% ============================================================
%% TRAJECTORY GENERATION
%% ============================================================

for p = 1:numPaths

    fprintf("Generating trajectory %d / %d\n",p,numPaths);

    %% ---------- RANDOM END (avoid duplicates) ----------

    validPath = false;

    while ~validPath

        q_end = [ ...
            q1_min + (q1_max-q1_min)*rand,...
            q2_min + (q2_max-q2_min)*rand,...
            q3_min + (q3_max-q3_min)*rand ];

        dq = abs(q_end - q_start);

        T_vel = max(1.875 * dq / v_max);
        T_acc = max(sqrt(5.77 * dq / a_max));

        T_min = max(T_vel, T_acc);

        T_rand = possible_T(randi(numel(possible_T)));

        T_total = max(T_min, T_rand);

        candidate = [q_start q_end T_total];

        if isempty(pathRecord)

            validPath = true;

        else

            diff = abs(pathRecord - candidate);

            if all(any(diff > 1e-4,2))
                validPath = true;
            end

        end
    end

    %% ---------- TIME VECTOR ----------

    t = (0:dt:T_total)';
    N = length(t);

    tau = t/T_total;

    %% ---------- MINIMUM JERK PROFILE ----------

    f   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    fd  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T_total;
    fdd = (60*tau - 180*tau.^2 + 120*tau.^3)/T_total^2;

    %% ---------- JOINT TRAJECTORY ----------

    q   = zeros(N,3);
    qd  = zeros(N,3);
    qdd = zeros(N,3);

    for j = 1:3

        dqj = q_end(j) - q_start(j);

        q(:,j)   = q_start(j) + dqj * f;
        qd(:,j)  = dqj * fd;
        qdd(:,j) = dqj * fdd;

    end

    %% ============================================================
    %% FORWARD KINEMATICS
    %% ============================================================

    xyz = zeros(N,3);

    for k = 1:N

        q1 = q(k,1);
        q2 = q(k,2);
        q3 = q(k,3);

        T01 = T_base * dh(alpha(1), a_dh(1), d_dh(1), q1);
        T12 = dh(alpha(2), a_dh(2), d_dh(2), q2);
        T23 = dh(alpha(3), a_dh(3), d_dh(3), q3);
        T34 = dh(alpha(4), a_dh(4), d_dh(4), q4);

        T04 = T01*T12*T23*T34;

        xyz(k,:) = T04(1:3,4)';

    end

    %% ---------- SAVE FILES ----------

    writematrix(q,...
        fullfile(angleDir,sprintf('path_%03d_angles.csv',p+450)));

    writematrix(xyz,...
        fullfile(xyzDir,sprintf('path_%03d_xyz.csv',p+450)));

    traj = [t q qd qdd];

    writematrix(traj,...
        fullfile(trajDir,sprintf('path_%03d_traj.csv',p+450)));

    %% ---------- SAVE PATH RECORD ----------

    newRecord = [q_start q_end T_total];

    pathRecord = [pathRecord; newRecord];

    writematrix(pathRecord,recordFile);

    %% ---------- LIVE PATH PLOT ----------

    plot3(xyz(:,1),xyz(:,2),xyz(:,3),'Color',rand(1,3),'LineWidth',1.5);
    drawnow

end

disp("All trajectories generated successfully.")

%% ============================================================
%% DH FUNCTION
%% ============================================================

function T = dh(alpha,a,d,theta)

T = [cos(theta) -sin(theta) 0 a;
     sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
     sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
     0 0 0 1];

end