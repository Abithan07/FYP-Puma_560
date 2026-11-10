% This code calculates Inverse dynamics of 3link PUMA 560 Arm

% 1. Symbolic declaration of T and q
syms T(alpha, len, ofs, theta)
syms q [1 3]

% 2. Simulation Parameters
g=9.81;
dt=0.02;        % simulation loop interval
Dur=6.0;        % simulation duration
t=[dt:dt:Dur];  % time line
N=Dur/dt;       % No of iterations in the simulation loops

% 3. Link parameters
a1=0;   a2=-pi/2;     a3=0;      % link twist alpha_{i-1}
L1=0;   L2=0;         L3=0.432;  % link length l_{i-1}
d1=0;   d2=0.233;     d3=-0.094; % Joint offset d_i

% 4. Link CoG
CoG2 = [0.068; 0.006; -0.016];
CoG3 = [0.000; -0.143; 0.014];

% 5. Base point as {1} is located at (0,0,0)
pb=[0;0;-0.671]; xb=pb(1); yb=pb(2); zb=pb(3);

% 6. Link masses
m1=0; m2=17.4; m3=4.8; % m1 is not required because link 1 only rotates around its z axis. Its inertia is seperately given

% 7. Link and motor inertia from the Stanford paper
Im1=1.14; Im2=4.71; Im3=0.83;
I1zz=0.35+Im1;
I2xx=0.130; I2yy=0.524; I2zz=0.539+Im2;
I3xx=0.066; I3yy=0.125; I3zz=0.086+Im3;

% 7.1 Inertia matrices
I1=[0 0 0; 0 0 0; 0 0 I1zz];
I2=[I2xx 0 0; 0 I2yy 0; 0 0 I2zz];
I3=[I3xx 0 0; 0 I3yy 0; 0 0 I3zz];

% 8. Homogeneous trasformation matrix - Modified DH convention
T(alpha, len, ofs, theta) =...
[   cos(theta)             -sin(theta)             0            len       ;
 sin(theta)*cos(alpha)  cos(theta)*cos(alpha)  -sin(alpha) -sin(alpha)*ofs;
 sin(theta)*sin(alpha)  cos(theta)*sin(alpha)  cos(alpha)   cos(alpha)*ofs;
        0                      0                      0            1     ];

% 9. Homogeneous transformation from base to other joints
T01 = T(a1, L1, d1, q1); % {0} to {1} HTM
T12 = T(a2, L2, d2, q2); % {1} to {2} HTM
T23 = T(a3, L3, d3, q3); % {2} to {3} HTM
T02 = T01*T12;           % {0} to {2} HTM
T03 = T02*T23;           % {0} to {3} HTM
 
% 10. Rotation Matrices
R01 = T01(1:3,1:3);
R02 = T02(1:3,1:3);
R03 = T03(1:3,1:3);

% 11. Angular velocity Jacobean
Jw1=[R01(:,3) [0; 0; 0] [0; 0; 0]];
Jw2=[R01(:,3)  R02(:,3) [0; 0; 0]];
Jw3=[R01(:,3)  R02(:,3)  R03(:,3)];

% 12. Link CoGs w. r. t. {0}
rc1=T01(1:3,4:4);           % Link1 CoG is considered (0,0,0) 
rc2=T02(1:3,4:4)+R02*CoG2;  % Link2 CoG w.r.t {2}
rc3=T03(1:3,4:4)+R03*CoG3;  % Link3 CoG w.r.t {3}

% 13. Potential energy of the three links
P = g*( m1*rc1(2) + m2*rc2(2) + m3*rc3(2));

% 14. Gravity Terms (syms)
G = [diff(P,q1) ; diff(P,q2); diff(P,q3)];

% 15. Partial differentiation of position vectors - Linear speeds at CoGs
v1q1=diff(rc1,q1); v1q2=diff(rc1,q2); v1q3=diff(rc1,q3);
v2q1=diff(rc2,q1); v2q2=diff(rc2,q2); v2q3=diff(rc2,q3);
v3q1=diff(rc3,q1); v3q2=diff(rc3,q2); v3q3=diff(rc3,q3);

% 16. Linear velocity Jacobean
Jv1=[v1q1 v1q2 v1q3]; Jv2=[v2q1 v2q2 v2q3]; Jv3=[v3q1 v3q2 v3q3];

% 17. Inertia Matrix (syms) of first three links + wrist
D = m1*transpose(Jv1)*Jv1 + transpose(Jw1)*R01*I1*transpose(R01)*Jw1 + ...
    m2*transpose(Jv2)*Jv2 + transpose(Jw2)*R02*I2*transpose(R02)*Jw2 + ...
    m3*transpose(Jv3)*Jv3 + transpose(Jw3)*R03*I3*transpose(R03)*Jw3;

d11=D(1,1);
d12=D(1,2);
d13=D(1,3);
d21=D(2,1);
d22=D(2,2);
d23=D(2,3);
d31=D(3,1);
d32=D(3,2);
d33=D(3,3);

% 18. Christoffel symbols of first kind (syms)
c111=0.5*diff(d11,q1);
c121=0.5*diff(d11,q2);
c131=0.5*diff(d11,q3);
c211=c121;
c221=-0.5*diff(d22,q1)+diff(d12,q2);
c231=0.5*(diff(d13,q2)+diff(d12,q3)-diff(d23,q1));
c311=c131;
c321=c231;
c331=diff(d13,q3)-0.5*diff(d33,q1);

c112=0.5*(2*diff(d21,q1)+diff(d11,q2));
c122=0.5*diff(d22,q1);
c132=0.5*( diff(d23,q1)+diff(d21,q3)-diff(d13,q2) );
c212=c122;
c222=0.5*diff(d22,q2);
c232=0.5*diff(d22,q3);
c312=c132;
c322=c232;
c332=diff(d23,q3)-0.5*diff(d33,q2);

c113=diff(d31,q1)-0.5*diff(d11,q3);
c123=0.5*(diff(d32,q1) + diff(d31,q2) -diff(d12,q3));
c133=0.5*diff(d33,q1);
c213=c123;
c223=diff(d32,q2)-0.5*diff(d22,q3);
c233=0.5*diff(d33,q2);
c313=c133;
c323=c233;
c333=0.5*diff(d33,q3);

% 19. Centrepetal torques (syms)
Cen=[c111 c221 c331;c112 c222 c332;c113 c223 c333];

% 20. Coriolis torque dq1.dq2 (syms)
Cor12=[c121+c211;c122+c212;c123+c213];

% 21. Coriolis torque dq1.dq3 (syms)
Cor13=[c131+c311;c132+c312;c133+c313];

% 22. Coriolis torque dq2.dq3 (syms)
Cor23=[c231+c321;c232+c322;c233+c323];

% 23. Joint Trajectory for Simulation - constant joint accelerations
ddq=[0.2; -0.1; 0.1]; q=[0;0;0]; dq=[0;0;0];

% 24. Simulation Loop
for k=[1:N]
q1=q(1); q2=q(2); q3=q(3);       % Joint position vector
d2q=[dq(1)^2; dq(2)^2; dq(3)^2]; % Joint velocity square vector

    if (k>N/2)
        ddq=[0.2; -0.1; 0.1];    % May change joint accelerations anytime
    end

% Symbolic to Numerical Calculations
nD = double(subs(D));           % Kinetic energy matrix
nCen = double(subs(Cen));       % Centripetal torque vector
nCor12 = double(subs(Cor12));   % Corriolis torques due to joint 1 and 2
nCor13 = double(subs(Cor13));   % Corriolis torques due to joint 1 and 3
nCor23 = double(subs(Cor23));   % Corriolis torques due to joint 2 and 3

nG = double(subs(G)); % Gravity vector

%L-E dynamics
tau = nD*ddq + nCen*d2q + nCor12*dq(1)*dq(2) +nCor13*dq(1)*dq(3) +...
      nCor23*dq(2)*dq(3)+ nG;

% Keep writing data to an array
tau1(k)=tau(1); tau2(k)=tau(2); tau3(k)=tau(3); % Joint torques 
nq1(k)=q(1); nq2(k)=q(2); nq3(k)=q(3);          % Joint positions
ndq1(k)=dq(1); ndq2(k)=dq(2); ndq3(k)=dq(3);    % Joint velocities

% Update joint position and velocities for the next loop
dq=dq+ddq*dt; % velocity vector
q=q+dq*dt;    % position vector

% Plot within the simulation loop all available data upto the present step
clf;

subplot(221); % Plot three angles
  plot(t(1:k),nq1(1:k),'-','LineWidth',1); hold on;...
  plot(t(1:k),nq2(1:k),'-','LineWidth',1); hold on;...
  plot(t(1:k),nq3(1:k),'-','LineWidth',1); hold on;...
  grid on; ylabel('$\bf{q}$ [rad]','Interpreter', 'latex');
  axis([0 Dur -2 3.5]); legend('$q_1$','$q_2$','$q_3$','Interpreter', 'latex','Location', 'northwest');

subplot(222); % plot joint velocities
  plot(t(1:k),ndq1(1:k),'-','LineWidth',1); hold on;...
  plot(t(1:k),ndq2(1:k),'-','LineWidth',1); hold on;...
  plot(t(1:k),ndq3(1:k),'-','LineWidth',1); hold on;...
  grid on; ylabel('$\dot{\bf{q}}$ [rad/s]','Interpreter', 'latex');
  axis([0 Dur -1 1.5]); legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','Interpreter', 'latex','Location', 'northwest');

subplot(223); % Plot joint torques
  plot(t(1:k),tau1(1:k),'-','LineWidth',1); hold on;...
  plot(t(1:k),tau2(1:k),'-','LineWidth',1); hold on;...
  plot(t(1:k),tau3(1:k),'-','LineWidth',1); hold on;...
  grid on; ylabel('$torque$ [Nm]','Interpreter', 'latex');
  axis([0 Dur -50 40]); legend('$\tau_1$','$\tau_2$','$\tau_3$','Interpreter', 'latex', 'Location', 'southwest');
  
subplot(224); % Arm configuration
  p1=double(subs(T01))*[0;0;0;1]; x1=p1(1); y1=p1(2); z1=p1(3); % {1}
  p2=double(subs(T02))*[0;0;0;1]; x2=p2(1); y2=p2(2); z2=p2(3); % {2}
  p2s=double(subs(T02))*[0.4318;0;0;1]; x2s=p2s(1); y2s=p2s(2); z2s=p2s(3);
  p3=double(subs(T03))*[0;0;0;1]; x3=p3(1); y3=p3(2); z3=p3(3); % {3}
  p4=double(subs(T03))*[0;-0.444;0;1]; x4=p4(1); y4=p4(2); z4=p4(3); % {4}
  nx4(k)=x4; ny4(k)=y4; nz4(k)=z4;
  
  plot3([xb x1],[yb y1],[zb z1],'k','LineWidth',2); hold on; %{0} to {1}
  plot3([x1 x2],[y1 y2],[z1 z2],'g','LineWidth',2); hold on; %L1
  plot3([x2 x2s x3],[y2 y2s y3],[z2 z2s z3],'m','LineWidth',2); hold on;%L2
  plot3([x3 x4],[y3 y4],[z3 z4],'b','LineWidth',2); hold on; %L3
  plot3(x1,y1,z1,'o','MarkerFaceColor','k','LineWidth',2); hold on;
  plot3(x2,y2,z2,'o','MarkerFaceColor','k','LineWidth',2); hold on;
  plot3(x3,y3,z3,'o','MarkerFaceColor','k','LineWidth',2); hold on;
  axis([-0.4 0.6 -0.8 0.6 -0.5 1.0]);
  xlabel('$x$ [m]','Interpreter', 'latex'); ylabel('$y$ [m]','Interpreter', 'latex');zlabel('$z$ [m]','Interpreter', 'latex');
  grid on;
  pause(0.0005)
end

% 25. Draw the end-effector path
for k=[1:N]
    subplot(224);
    plot3(nx4(k),ny4(k),nz4(k),'.r');
end