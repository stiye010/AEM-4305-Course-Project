%% Project Part 6
% mainSimFullMission
% This simulates a full mission scenario using Nadir Pointing Control,
% Communication Pointing Control, and Sun Pointing Control

close all; clear; clc

% Load in Parameters and Initial Conditions
params = loadParameters();

%% Nadir Pointing
% Simulate
[t_NP,x_NP] = ode45(@(t,x) f2BPGravGradNP(x, t, params), ...
    params.tspanNP, params.x0, params.odeOptions3);

r_NP = x_NP(:,1:3);
v_NP = x_NP(:,4:6);
q_NP = x_NP(:,7:10);
n_NP = x_NP(:,7);
e_NP = x_NP(:,8:10);
w_NP = x_NP(:,11:13);

% Sensor Measurements
[rwc_n_NP,rsc_n_NP,rwc_b_NP,rsc_b_NP] = genMeasurements(x_NP);

% Euler Angle Errors
C_bn_est_NP = DCMEstimate(rwc_n_NP,rsc_n_NP,rwc_b_NP,rsc_b_NP);

% Control Torque
N_NP = length(t_NP);
tau_u_NP = zeros(N_NP,3);
q_br_est_NP = zeros(N_NP,4);
I = params.Ib;
mu = params.mu_moon;
k_d = 0.01;
Kd = [k_d 0 0;...
    0 k_d 0;...
    0 0 k_d];
kp = 0.03;

for i = 1:N_NP
    w_i = w_NP(i,:)';
    r_i = r_NP(i,:)';
    v_i = v_NP(i,:)';
    q_i = q_NP(i,:)';

    % DCMs
    C_bn_i = quaternion2DCM(q_i);

    C_rdn = dcmRd2N(r_i,v_i);

    C_br_est_i = C_bn_est_NP{i} * C_rdn';

    % Angular velocities
    w_rdn_rd = wRdN(r_i,params);

    w_rdn_n = C_rdn' * w_rdn_rd;

    w_rdn_b = C_bn_est_NP{i} * w_rdn_n;

    w_bn_b_est = w_i;

    w_brd_b = w_bn_b_est - w_rdn_b;

    % Quaternion 
    q_br_est_NP(i,:) = DCM2quaternion(C_br_est_i);
    e_br = q_br_est_NP(i,2:4)';

    % Torques
    tau_gg = 3*mu / (r_i'*r_i)^(5/2) * Cross(C_bn_i*r_i) * I*C_bn_i*r_i;
    tau_u_NP(i,:) = -kp*e_br - Kd*w_brd_b - I*Cross(w_bn_b_est)*w_rdn_b ...
        + Cross(w_rdn_b)*I*w_bn_b_est - tau_gg;
end
%% Communication Pointing
x_CP_0 = x_NP(end,:);

rgwn = buildGatewayPosInterp();

% Simulate
[t_CP,x_CP] = ode45(@(t,x) f2BPGravGradCP(x, t, params, rgwn), ...
    params.tspanCP, x_CP_0, params.odeOptions3);

r_CP = x_CP(:,1:3);
v_CP = x_CP(:,4:6);
q_CP = x_CP(:,7:10);
n_CP = x_CP(:,7);
e_CP = x_CP(:,8:10);
w_CP = x_CP(:,11:13);

% Sensor Measurements
[rwc_n_CP,rsc_n_CP,rwc_b_CP,rsc_b_CP] = genMeasurements(x_CP);

% Euler Angle Errors
C_bn_est_CP = DCMEstimate(rwc_n_CP,rsc_n_CP,rwc_b_CP,rsc_b_CP);

% Control Torque
N_CP = length(t_CP);
tau_u_CP = zeros(N_CP,3);
q_br_est_CP = zeros(N_CP,4);

w_rcn_rc = wRcN(r_CP,t_CP,rgwn);

for i = 1:N_CP
    w_i = w_CP(i,:)';
    r_i = r_CP(i,:)';
    v_i = v_CP(i,:)';
    q_i = q_CP(i,:)';
    rgw_n = rgwn(t_CP(i));

    % DCMs
    C_bn_i = quaternion2DCM(q_i);

    C_rcn = dcmRc2N(r_i,rgw_n);

    C_br_est_i = C_bn_est_CP{i} * C_rcn';

    % Angular velocities
    w_rcn_rc_i = w_rcn_rc(i,:)'; 

    w_rcn_n = C_rcn' * w_rcn_rc_i;

    w_rcn_b = C_bn_est_CP{i} * w_rcn_n;

    w_bn_b_est = w_i;

    w_brc_b = w_bn_b_est - w_rcn_b;

    % Quaternion 
    q_br_est_CP(i,:) = DCM2quaternion(C_br_est_i);
    e_br = q_br_est_CP(i,2:4)';

    % Torques
    tau_gg = 3*mu / (r_i'*r_i)^(5/2) * Cross(C_bn_i*r_i) * I*C_bn_i*r_i;
    tau_u_CP(i,:) = -kp*e_br - Kd*w_brc_b - I*Cross(w_bn_b_est)*w_rcn_b ...
        + Cross(w_rcn_b)*I*w_bn_b_est - tau_gg;
end
%% Sun Pointing
x_SP_0 = x_CP(end,:);

% Simulate
[t_SP,x_SP] = ode45(@(t,x) f2BPGravGradSP(x, t, params), ...
    params.tspanSP, x_SP_0, params.odeOptions3);

r_SP = x_SP(:,1:3);
v_SP = x_SP(:,4:6);
q_SP = x_SP(:,7:10);
n_SP = x_SP(:,7);
e_SP = x_SP(:,8:10);
w_SP = x_SP(:,11:13);

% Sensor Measurements
[rwc_n_SP,rsc_n_SP,rwc_b_SP,rsc_b_SP] = genMeasurements(x_SP);

% Euler Angle Errors
C_bn_est_SP = DCMEstimate(rwc_n_SP,rsc_n_SP,rwc_b_SP,rsc_b_SP);

C_rsn = dcmRs2N();

% Control Torque
N_SP = length(t_SP);
tau_u_SP = zeros(N_SP,3);
q_br_est_SP = zeros(N_SP,4);

for i = 1:N_SP
    w_i = w_SP(i,:)';
    r_i = r_SP(i,:)';
    q_i = q_SP(i,:)';
    C_bn_i = quaternion2DCM(q_i);
    C_br_est_i = C_bn_est_SP{i} * C_rsn';
    q_br_est_SP(i,:) = DCM2quaternion(C_br_est_i);

    e_br = q_br_est_SP(i,2:4)';

    tau_gg = 3*mu / (r_i'*r_i)^(5/2) * Cross(C_bn_i*r_i) * I*C_bn_i*r_i;
    tau_u_SP(i,:) = -kp*e_br - Kd*w_i - tau_gg;
end

%% Concatenate Data
t_total = [t_NP; t_CP; t_SP];
q_br_total = [q_br_est_NP; q_br_est_CP; q_br_est_SP];
tau_u_total = [tau_u_NP; tau_u_CP; tau_u_SP];

%% Plotting
% Control Torque
figure
subplot(3,1,1)
plot(t_total,tau_u_total(:,1),'k',"LineWidth",1.2);
grid on; hold on;
xline(2500,'r--');
xline(5000,'b--');
ylabel('\tau_u_1 (Nm)');
subplot(3,1,2)
plot(t_total,tau_u_total(:,2),'k',"LineWidth",1.2);
grid on; hold on;
xline(2500,'r--');
xline(5000,'b--');
ylabel('\tau_u_2 (Nm)');
subplot(3,1,3)
plot(t_total,tau_u_total(:,3),'k',"LineWidth",1.2);
grid on; hold on;
xline(2500,'r--');
xline(5000,'b--');
ylabel('\tau_u_3 (Nm)'); xlabel('Time (s)');
legend('Control Torque Component','NP to CP Transition', ...
    'CP to SP Transition');

% Eta_br and Epsilon_br
figure
subplot(4,1,1)
plot(t_total,q_br_total(:,1),'k',"LineWidth",1.2);
grid on; hold on;
xline(2500,'r--');
xline(5000,'b--');
ylabel('\eta_{br}');
subplot(4,1,2)
plot(t_total,q_br_total(:,2),'k',"LineWidth",1.2);
grid on; hold on;
xline(2500,'r--');
xline(5000,'b--');
ylabel('\epsilon_{br}_1');
subplot(4,1,3)
plot(t_total,q_br_total(:,3),'k',"LineWidth",1.2);
grid on; hold on;
xline(2500,'r--');
xline(5000,'b--');
ylabel('\epsilon_{br}_2');
subplot(4,1,4)
plot(t_total,q_br_total(:,4),'k',"LineWidth",1.2);
grid on; hold on;
xline(2500,'r--');
xline(5000,'b--');
ylabel('\epsilon_{br}_3'); xlabel('Time (s)');
legend('Quaternion Component','NP to CP Transition', ...
    'CP to SP Transition');