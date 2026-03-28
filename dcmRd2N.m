function [DCM] = dcmRd2N(rn,rn_dot)
% dcmRd2N computes the DCM that describes the nadir-pointing frame 
% relative to the inertial frame given the spacecrafts position and 
% velocity in the inertial frame

rd_2n = Cross(rn)*rn_dot / norm(Cross(rn)*rn_dot);
rd_3n = rn / norm(rn);
rd_1n = Cross(rd_2n) * rd_3n;

DCM = [rd_1n'; rd_2n'; rd_3n'];

end