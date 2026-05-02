%% Project Part 4 and 5
% mainSim.m
% numerically integrates a spacecrafts orbital and attitude motion 
% full 6 degree-of-freedom simulation

close all; clear; clc

% Load in Parameters and Initial Conditions
params = loadParameters();

% Simulate
[t,x] = ode45(@(t,x) f2BPGravGrad(x,params), ...
    params.tspan, params.x0, params.odeOptions3);

r = x(:,1:3);
v = x(:,4:6);
q = x(:,7:10);
n = x(:,7);
e = x(:,8:10);
w = x(:,11:13);

% Quaternion norm error
q_norm = sum(q.^2,2);
q_err = q_norm - 1;

% Energy
E = computeSc2BPGGEnergy(x,params);
E_diff = E - E(1);

% Euler Angles
eul = quats2EulAng321(q);

% Sensor Measurements
[rwc_n,rsc_n,rwc_b,rsc_b] = genMeasurements(x);

% Euler Angle Errors
C_TRIAD = DCMEstimate(rwc_n,rsc_n,rwc_b,rsc_b);

C_eb = DCMError(C_TRIAD,q);

N = size(x,1);
eul_err = zeros(N,3);

for i = 1:N
    eul_err(i,:) = DCM2euler321(C_eb{i});
end

% Spacecraft Position Relative to Gateway Resolved in Communication Frame
rgwn = buildGatewayPosInterp();

r_gs_rc = posgsN2Rc(r,t,rgwn);

% Angular Velocity Communication Frame
w_rcn_rc = wRcN(r,t,rgwn);

%% Plots
% Position
figure
plot3(r(:,1),r(:,2),r(:,3),"LineWidth",1.2);
grid on;
xlabel('r_1 [km]'); ylabel('r_2 [km]'); zlabel('r_3 [km]');

% Velocity
figure
plot3(v(:,1),v(:,2),v(:,3),"LineWidth",1.2);
grid on;
xlabel('v_1 [km/s]'); ylabel('v_2 [km/s]'); zlabel('v_3 [km/s]');

% Eta and Epsilon
figure
subplot(4,1,1)
plot(t,n,"LineWidth",1.2);
grid on;ylabel('\eta');
subplot(4,1,2)
plot(t,e(:,1),"LineWidth",1.2);
grid on; ylabel('\epsilon_1');
subplot(4,1,3)
plot(t,e(:,2),"LineWidth",1.2);
grid on; ylabel('\epsilon_2');
subplot(4,1,4)
plot(t,e(:,3),"LineWidth",1.2);
grid on; ylabel('\epsilon_3'); xlabel('Time (s)');

% Quaternion error
figure
plot(t,q_err,"LineWidth",1.2);
grid on;
xlabel('Time (s)'); ylabel('q^Tq - 1')

% Angular velocity
figure
subplot(3,1,1)
plot(t,w(:,1),"LineWidth",1.2);
grid on; ylabel('\omega_1 (rad/s)');
subplot(3,1,2)
plot(t,w(:,2),"LineWidth",1.2);
grid on; ylabel('\omega_2 (rad/s)');
subplot(3,1,3)
plot(t,w(:,3),"LineWidth",1.2);
grid on; ylabel('\omega_3 (rad/s)'); xlabel('Time (s)');

% Energy
figure
plot(t,E_diff,"LineWidth",1.2);
grid on;
xlabel('Time (s)'); ylabel('E(t) - E(0)');

% Euler Angles
figure
subplot(3,1,1)
plot(t,rad2deg(eul(:,1)),"LineWidth",1.2);
grid on; ylabel('\psi (deg)');
subplot(3,1,2)
plot(t,rad2deg(eul(:,2)),"LineWidth",1.2);
grid on; ylabel('\theta (deg)');
subplot(3,1,3)
plot(t,rad2deg(eul(:,3)),"LineWidth",1.2);
grid on; ylabel('\phi (deg)'); xlabel('Time (s)');

% Euler Angles Error
figure
subplot(3,1,1)
plot(t,rad2deg(eul_err(:,1)),"LineWidth",1.2);
grid on; ylabel('\psi_{err} (deg)');
subplot(3,1,2)
plot(t,rad2deg(eul_err(:,2)),"LineWidth",1.2);
grid on; ylabel('\theta_{err} (deg)');
subplot(3,1,3)
plot(t,rad2deg(eul_err(:,3)),"LineWidth",1.2);
grid on; ylabel('\phi_{err} (deg)'); xlabel('Time (s)');

% Position: s/c relative to Gateway resolved in communication frame
% first 1000 time steps
figure
subplot(3,1,1)
plot(t(1:1000),r_gs_rc(1:1000,1),"LineWidth",1.2);
grid on; ylabel('r_{gs,r^c,1} (km)');
subplot(3,1,2)
plot(t(1:1000),r_gs_rc(1:1000,2),"LineWidth",1.2);
grid on; ylabel('r_{gs,r^c,2} (km)');
subplot(3,1,3)
plot(t(1:1000),r_gs_rc(1:1000,3),"LineWidth",1.2);
grid on; ylabel('r_{gs,r^c,3} (km)'); xlabel('Time (s)')

% Angular Velocity Communication Frame
% first 1000 time steps
figure
subplot(3,1,1)
plot(t(1:1000),w_rcn_rc(1:1000,1),"LineWidth",1.2);
grid on; ylabel('w_{r^cn,r^c,1} (rad/s)');
subplot(3,1,2)
plot(t(1:1000),w_rcn_rc(1:1000,2),"LineWidth",1.2);
grid on; ylabel('w_{r^cn,r^c,2} (rad/s)');
subplot(3,1,3)
plot(t(1:1000),w_rcn_rc(1:1000,3),"LineWidth",1.2);
grid on; ylabel('w_{r^cn,r^c,3} (rad/s)'); xlabel('Time (s)')