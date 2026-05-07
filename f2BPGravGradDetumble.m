function [xdot] = f2BPGravGradDetumble(x, t, params)
% f2BPGravGradDetumble takes in the state and the parameters struct and 
% outputs the equations of motion xdot for detumbling

% This uses f2BP to get r_dot and v_dot 
% Also uses fAtt to get q_dot and w_dot

% r_dot and v_dot from f2BP
rdot_vdot = f2BP(x,params);

% Define variables form params struct
I = params.Ib;
mu = params.mu_moon;

% Define variables used to calculate external torque
r_sw = x(1:3);
q = x(7:10);
w = x(11:13);
C_bn = quaternion2DCM(q);
k_d = 0.005;
Kd = [k_d 0 0;...
      0 k_d 0;...
      0 0 k_d];

w_est = rateGyroNoisy(w,t);

% External torque equation
tau_gg = 3*mu / (r_sw'*r_sw)^(5/2) * Cross(C_bn*r_sw) * I*C_bn*r_sw;
tau_u = -Kd*w_est - tau_gg;

tau_total = tau_u + tau_gg;

% q_dot and w_dot from fAtt
qdot_wdot = fAtt(x,tau_total,params);

% Putting rdot_vdot and qdot_wdot together to get xdot
xdot = [rdot_vdot; qdot_wdot];
end