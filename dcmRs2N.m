function [C] = dcmRs2N()
% This function defines the DCM describing the Sun-pointing frame relative
% to the inertial frame
C = [-1 0 0;...
      0 0 1;...
      0 1 0];
end