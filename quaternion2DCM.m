function [DCM] = quaternion2DCM(q)
% quaternion2DCM takes in a quaternion and converts it to a DCM
% uses Cross function created in HW 1

n = q(1);
E = q(2:4);

DCM = (n^2-E'*E)*eye(3) + 2*E*E' - 2*n*Cross(E);
end