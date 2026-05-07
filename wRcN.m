function [w_rcn_rc] = wRcN(rsw_n,t,rgwn)
% wrRcN takes in the position and time data of the spacecraft relative
% to the moon in the inertial frame and the gateway position interpolant in
% the inertial frame.
% It outputs the angular velocity of the communication frame relative to
% the inertial frame resolved in the communication frame

N = length(t);

% dt = t(2) - t(1);
dt = 1;

w_rcn_rc = zeros(N,3);

for i = 1:N-1
    C_rcn_1 = dcmRc2N(rsw_n(i)',rgwn(t(i)));
    C_rcn_2 = dcmRc2N(rsw_n(i+1)',rgwn(t(i+1)));

    C_rcn_dot = (C_rcn_2 - C_rcn_1) / dt;

    wx = C_rcn_dot * C_rcn_1';

    w1 = wx(3,2);
    w2 = wx(1,3);
    w3 = wx(2,1);
    
    w_rcn_rc(i,:) = [w1 w2 w3];
end
end