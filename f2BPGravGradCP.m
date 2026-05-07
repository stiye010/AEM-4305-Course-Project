function [xdot] = f2BPGravGradCP(x, t, params, rgwn)
% f2BPGravGradCP takes in the state and the parameters struct and 
% outputs the equations of motion xdot for communication pointing

% This uses f2BP to get r_dot and v_dot 
% Also uses fAtt to get q_dot and w_dot

% r_dot and v_dot from f2BP
rdot_vdot = f2BP(x,params);

% Define variables form params struct
I = params.Ib;
mu = params.mu_moon;

% Define variables
rwc_n = x(1:3);
q = x(7:10);
w = x(11:13);
C_bn = quaternion2DCM(q);
k_d = 0.01;
Kd = [k_d 0 0;...
    0 k_d 0;...
    0 0 k_d];
kp = 0.03;

dt = 1;

rgw_n = rgwn(t);
rgwn_tdt = rgwn(t+dt);

% Sun position (same as your function)
rsw_n = [0; 150000000; 0];

% Sun-to-spacecraft vector
rsc_n = rsw_n + rwc_n;

% Noisy measurements
rsc_b = sunSensorNoisy(rsc_n, C_bn);
rwc_b = lunarSensorNoisy(rwc_n, C_bn);

% DCMs
C_bn_est = TRIAD(rsc_n,rwc_n,rsc_b,rwc_b);

rwc_n_tdt = rwc_n + rdot_vdot(1:3) * dt;

C_rcn = dcmRc2N(rwc_n,rgw_n);
C_rcn_tdt = dcmRc2N(rwc_n_tdt,rgwn_tdt);

C_br_est = C_bn_est * C_rcn';

% quaternion
q_br_est = DCM2quaternion(C_br_est);

e_br = q_br_est(2:4);

% Angular Velocities
C_rcn_dot = (C_rcn_tdt - C_rcn) / dt;

wx = C_rcn_dot * C_rcn';

w1 = wx(3,2);
w2 = wx(1,3);
w3 = wx(2,1);

w_rcn_rc = [w1; w2; w3];

w_rcn_n = C_rcn' * w_rcn_rc;

w_rcn_b = C_bn_est * w_rcn_n;

w_bn_b_est = rateGyroNoisy(w,t);

w_brc_b = w_bn_b_est - w_rcn_b;

% External torque equation
tau_gg = 3*mu / (rwc_n'*rwc_n)^(5/2) * Cross(C_bn*rwc_n) * I*C_bn*rwc_n;
tau_u = -kp*e_br - Kd*w_brc_b - I*Cross(w_bn_b_est)*w_rcn_b ...
    + Cross(w_rcn_b)*I*w_bn_b_est - tau_gg;

tau_total = tau_u + tau_gg;

% q_dot and w_dot from fAtt
qdot_wdot = fAtt(x,tau_total,params);

% Putting rdot_vdot and qdot_wdot together to get xdot
xdot = [rdot_vdot; qdot_wdot];
end