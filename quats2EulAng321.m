function [eul] = quats2EulAng321(q)
% quats2EulAng321 takes in the quaternions from simulation and outputs the
% corresponding 321 euler angles

N = size(q,1);
eul = zeros(N,3);

for i = 1:N
    qi = q(i,:)';
    C = quaternion2DCM(qi);
    eul(i,:) = DCM2euler321(C);
end