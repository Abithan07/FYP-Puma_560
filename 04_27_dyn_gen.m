clc; clear; close all;

%% Trajectory folders
% cd('C:\Users\Priyankan\Desktop\Trajectory Gen\Jan 31 Trape\Dataset');

cd('C:\Users\Priyankan\Desktop\Trajectory Gen\Dataset 04-27_n');
% cd('C:\Users\ROG\Desktop\Trajectory Gen\Datasets\Paths_1');
% C:\Users\ROG\Desktop\Trajectory Gen\100 angles\100_trajectories
inputFolder  = 'Trajectories';
outputFolder = 'Joint_states';
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end
files = dir(fullfile(inputFolder,'*.csv'));

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

%% 8. Parallel computation of torques (row-wise output)
parfor f = 1:length(files)
    data = readmatrix(fullfile(inputFolder,files(f).name));
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
    
    % [~, baseName, ext] = fileparts(fileName);
    % newName = sprintf('%s_joint_states%s', baseName, ext);
    % 
    % % Build full output path
    % outputFile = fullfile(newName);
    % writecell(outCell, outputFile);
    % 
    % fprintf('Processed %s\n', fileName);
    % Create new filename by replacing "_trajectory" with "_joint_states"
    newName = strrep(files(f).name, '_traj', '_joint_states');

    % Build full output path
    outputFile = fullfile(outputFolder, newName);
    writecell(outCell, outputFile);

    fprintf('Processed %s\n', files(f).name);
end
% disp('All trajectories processed and row-wise data saved.');