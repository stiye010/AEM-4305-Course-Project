function [C_eb] = DCMError(C_TRIAD,q)
% DCMError takes in the estimated DCM's from DCMEstimate and the 
% quaternions from the state. Using the quaternions it computes the true 
% DCM. This function computes and outputs the error between the estimated 
% DCM and true DCM

N = size(q,1);
C_eb = cell(N,1);

for i = 1:N
C_bn = quaternion2DCM(q(i,:)');
C_nb = C_bn';

C_eb{i} = C_TRIAD{i} * C_nb;
end