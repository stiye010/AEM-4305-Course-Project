function [xdot] = f2BPGravGrad(x,params)
% f2BPGravGrad takes in the state and the parameters struct and outputs
% the equations of motion xdot

% This uses f2BP to get r_dot and v_dot 
% Also uses fAtt to get q_dot and w_dot

% r_dot and v_dot from f2BP
rdot_vdot = f2BP(x,params);

% Define variables form params struct
I = params.Ib;
mu = params.mu_moon;

% Define varibales used to calculate external torque
r_sw = x(1:3);
q = x(7:10);
C_bn = quaternion2DCM(q);

% External torque equation
tau = 3*mu / (r_sw'*r_sw)^(5/2) * Cross(C_bn*r_sw) * I*C_bn*r_sw;

% q_dot and w_dot from fAtt
qdot_wdot = fAtt(x,tau,params);

% Puttting rdot_vdot and qdot_wdot together to get xdot
xdot = [rdot_vdot; qdot_wdot];
end