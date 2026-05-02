function [r_gs_rc] = posgsN2Rc(r_sw_n,t,rgwn)
% posgsN2Rc takes in the position and time data of the spacecraft relative
% to the moon in the inertial frame and the gateway position interpolant in
% the inertial frame.
% It outputs the position of the spacecraft relative to the gateway 
% resolved in the communication frame

N = length(t);

r_gs_rc = zeros(N,3);

for i = 1:N
rsw_n = r_sw_n(i,:)';
rgw_n = rgwn(t(i));

r_gs_n = rgw_n - rsw_n;

C_rcn = dcmRc2N(rsw_n,rgw_n);

r_gs_rc(i,:) = (C_rcn*r_gs_n)';
end
end