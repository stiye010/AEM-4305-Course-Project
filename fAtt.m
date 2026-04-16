function [xdot] = fAtt(x,tau,params)
% fAtt takes in the state and outputs the equations of motion
% Input state x and external torque, uses loadParameters and the 
% quaternionKinematics function

I = params.Ib;
q = x(7:10);
w = x(11:13);
wx = Cross(w);

q_dot = quaternionKinematics(w,q);

w_dot = I^(-1)*(tau - wx*I*w);

xdot = [q_dot; w_dot];
end