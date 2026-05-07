%% Project Part 6
% Main Sim for Questions 3b
% numerically integrates a spacecrafts orbital and attitude motion 
% full 6 degree-of-freedom simulation
% With Communication Pointing Control Law

close all; clear; clc

% Load in Parameters and Initial Conditions
params = loadParameters();

rgwn = buildGatewayPosInterp();

% Simulate
[t,x] = ode45(@(t,x) f2BPGravGradCP(x, t, params, rgwn), ...
    params.tspan, params.x0, params.odeOptions3);

r = x(:,1:3);
v = x(:,4:6);
q = x(:,7:10);
n = x(:,7);
e = x(:,8:10);
w = x(:,11:13);

% Sensor Measurements
[rwc_n,rsc_n,rwc_b,rsc_b] = genMeasurements(x);

% Euler Angle Errors
C_bn_est = DCMEstimate(rwc_n,rsc_n,rwc_b,rsc_b);

% Control Torque
N = length(t);
tau_u = zeros(N,3);
q_br_est = zeros(N,4);
I = params.Ib;
mu = params.mu_moon;
k_d = 0.01;
Kd = [k_d 0 0;...
    0 k_d 0;...
    0 0 k_d];
kp = 0.03;

w_rcn_rc = wRcN(r,t,rgwn);

for i = 1:N
    w_i = w(i,:)';
    r_i = r(i,:)';
    v_i = v(i,:)';
    q_i = q(i,:)';
    rgw_n = rgwn(t(i));

    % DCMs
    C_bn_i = quaternion2DCM(q_i);

    C_rcn = dcmRc2N(r_i,rgw_n);

    C_br_est_i = C_bn_est{i} * C_rcn';

    % Angular velocities
    w_rcn_rc_i = w_rcn_rc(i,:)'; 

    w_rcn_n = C_rcn' * w_rcn_rc_i;

    w_rcn_b = C_bn_est{i} * w_rcn_n;

    w_bn_b_est = w_i;

    w_brc_b = w_bn_b_est - w_rcn_b;

    % Quaternion 
    q_br_est(i,:) = DCM2quaternion(C_br_est_i);

    e_br = q_br_est(i,2:4)';

    % Torques
    tau_gg = 3*mu / (r_i'*r_i)^(5/2) * Cross(C_bn_i*r_i) * I*C_bn_i*r_i;
    tau_u(i,:) = -kp*e_br - Kd*w_brc_b - I*Cross(w_bn_b_est)*w_rcn_b ...
        + Cross(w_rcn_b)*I*w_bn_b_est - tau_gg;
end
%% Plotting
% Control Torque
figure
subplot(3,1,1)
plot(t,tau_u(:,1),"LineWidth",1.2);
grid on; ylabel('\tau_u_1 (Nm)');
subplot(3,1,2)
plot(t,tau_u(:,2),"LineWidth",1.2);
grid on; ylabel('\tau_u_2 (Nm)');
subplot(3,1,3)
plot(t,tau_u(:,3),"LineWidth",1.2);
grid on; ylabel('\tau_u_3 (Nm)'); xlabel('Time (s)');

% Eta_br and Epsilon_br
figure
subplot(4,1,1)
plot(t,q_br_est(:,1),"LineWidth",1.2);
grid on;ylabel('\eta_{br^c}');
subplot(4,1,2)
plot(t,q_br_est(:,2),"LineWidth",1.2);
grid on; ylabel('\epsilon_{br^c}_1');
subplot(4,1,3)
plot(t,q_br_est(:,3),"LineWidth",1.2);
grid on; ylabel('\epsilon_{br^c}_2');
subplot(4,1,4)
plot(t,q_br_est(:,4),"LineWidth",1.2);
grid on; ylabel('\epsilon_{br^c}_3'); xlabel('Time (s)');