function [x_dot] = fAtt(x)
% fAtt takes in the state and outputs the equations of motion
% Input state x, uses loadParameters and the quaternionKinematics function

params = loadParameters;
I = params.Ib;
n = x(1);
E = x(2:4);
w = x(5:7);
wx = Cross(w);

q_dot = quaternionKinematics(w,[n;E]);

w_dot = I^(-1)*(-wx)*I*w;

x_dot = [q_dot; w_dot];
end