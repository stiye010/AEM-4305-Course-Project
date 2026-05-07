function [xdot] = f2BPGravGradSP(x, t, params)
% f2BPGravGradSP takes in the state and the parameters struct and 
% outputs the equations of motion xdot for sun pointing

% This uses f2BP to get r_dot and v_dot 
% Also uses fAtt to get q_dot and w_dot

% r_dot and v_dot from f2BP
rdot_vdot = f2BP(x,params);

% Define variables form params struct
I = params.Ib;
mu = params.mu_moon;

% Define variables used to calculate external torque
rwc_n = x(1:3);
q = x(7:10);
w = x(11:13);
C_bn = quaternion2DCM(q);
k_d = 0.01;
Kd = [k_d 0 0;...
    0 k_d 0;...
    0 0 k_d];
kp = 0.03;

w_est = rateGyroNoisy(w,t);

% Sun position (same as your function)
rsw_n = [0; 150000000; 0];

% Sun-to-spacecraft vector
rsc_n = rsw_n + rwc_n;

% Noisy measurements
rsc_b = sunSensorNoisy(rsc_n, C_bn);
rwc_b = lunarSensorNoisy(rwc_n, C_bn);

C_bn_est = TRIAD(rsc_n,rwc_n,rsc_b,rwc_b);

C_rsn = dcmRs2N();

C_br_est = C_bn_est * C_rsn';

q_br_est = DCM2quaternion(C_br_est);

e_br = q_br_est(2:4);

% External torque equation
tau_gg = 3*mu / (rwc_n'*rwc_n)^(5/2) * Cross(C_bn*rwc_n) * I*C_bn*rwc_n;
tau_u = -kp*e_br - Kd*w_est - tau_gg;

tau_total = tau_u + tau_gg;

% q_dot and w_dot from fAtt
qdot_wdot = fAtt(x,tau_total,params);

% Putting rdot_vdot and qdot_wdot together to get xdot
xdot = [rdot_vdot; qdot_wdot];
end