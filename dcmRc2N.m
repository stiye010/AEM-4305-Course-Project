function [C_rcn] = dcmRc2N(rsw_n,rgw_n)
% This function takes in the spacecraft and Gateway position components in
% the inertial and computes the DCM hat describes the communication frame
% relative to the inertial frame.

n3 = [0;0;1];
r_c3_n = (rgw_n - rsw_n) / norm(rgw_n - rsw_n);
r_c1_n = Cross(rgw_n - rsw_n)*n3 / norm(Cross(rgw_n - rsw_n)*n3);
r_c2_n = Cross(r_c3_n) * r_c1_n;

C_rcn = [r_c1_n'; r_c2_n'; r_c3_n'];
end