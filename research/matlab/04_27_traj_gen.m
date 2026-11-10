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
%% ++++++++++ Change These before run as per need. +++++

pathStartId = 601;

% ----------- END LIMITS -----------------

q1_min_deg = -150;
q1_max_deg = 150;

q2_min_deg = -215;
q2_max_deg = 45;

q3_min_deg = -35;
q3_max_deg = 215;

%% ++++++++++++++++++++++++++++++++++++++++

v_max = 2;
a_max = 7;

%% ===================== OUTPUT DIRECTORIES =====================

baseDir = 'C:\Users\Priyankan\Desktop\Trajectory Gen\Dataset 04-27_n';

angleDir = fullfile(baseDir,'Angles');
xyzDir   = fullfile(baseDir,'XYZ');
trajDir  = fullfile(baseDir,'Trajectories');

if ~exist(angleDir,'dir'); mkdir(angleDir); end
if ~exist(xyzDir,'dir'); mkdir(xyzDir); end
if ~exist(trajDir,'dir'); mkdir(trajDir); end

%% ===================== PATH RECORD =====================

recordDir = 'C:\Users\Priyankan\Desktop\Trajectory Gen\Joint States All';
if ~exist(recordDir,'dir'); mkdir(recordDir); end

recordFile = fullfile(recordDir,'all_paths_0.csv');

if exist(recordFile,'file')
    pathRecord = readmatrix(recordFile);
    if ~isempty(pathRecord) && size(pathRecord,2) == 7
        pathRecord = [ (1:size(pathRecord,1))' rad2deg(pathRecord(:,4:6)) pathRecord(:,7) ];
    end
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

q_start_deg = [0 45 135];
q_start = deg2rad(q_start_deg);
q4_deg = 0;
q4 = deg2rad(q4_deg);

%% ===================== VISUALIZATION =====================

figPath = figure('Color','w'); hold on; grid on; axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Generated PUMA-560 Paths')
view(3)

figEnd = figure('Color','w'); hold on; grid on; axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Generated PUMA-560 End Points')
view(3)

%% ============================================================
%% TRAJECTORY GENERATION
%% ============================================================

for p = 1:numPaths

    fprintf("Generating trajectory %d / %d\n",p,numPaths);
    pathId = pathStartId + p - 1;

    %% ---------- RANDOM END (avoid duplicates) ----------

    validPath = false;

    while ~validPath

        q_end_deg = [ ...
            q1_min_deg + (q1_max_deg-q1_min_deg)*rand,...
            q2_min_deg + (q2_max_deg-q2_min_deg)*rand,...
            q3_min_deg + (q3_max_deg-q3_min_deg)*rand ];

        q_end = deg2rad(q_end_deg);

        dq = abs(q_end - q_start);

        T_vel = max(1.875 * dq / v_max);
        T_acc = max(sqrt(5.77 * dq / a_max));

        T_min = max(T_vel, T_acc);

        T_rand = possible_T(randi(numel(possible_T)));

        T_total = max(T_min, T_rand);

        candidate = [pathId q_end_deg T_total];

        if isempty(pathRecord)

            validPath = true;

        else
            diff = abs(pathRecord(:,2:5) - candidate(2:5));

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
        fullfile(angleDir,sprintf('path_%03d_angles.csv',pathId)));

    writematrix(xyz,...
        fullfile(xyzDir,sprintf('path_%03d_xyz.csv',pathId)));

    traj = [t q qd qdd];

    writematrix(traj,...
        fullfile(trajDir,sprintf('path_%03d_traj.csv',pathId)));

    %% ---------- SAVE PATH RECORD ----------

    newRecord = [pathId q_end_deg T_total];

    pathRecord = [pathRecord; newRecord];

    writematrix(pathRecord,recordFile);

    %% ---------- LIVE PATH PLOT ----------

    figure(figPath);
    plot3(xyz(:,1),xyz(:,2),xyz(:,3),'Color',rand(1,3),'LineWidth',1.5);
    drawnow

    figure(figEnd);
    plot3(xyz(end,1),xyz(end,2),xyz(end,3),'o','MarkerFaceColor',rand(1,3),'MarkerEdgeColor','k','MarkerSize',3);
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