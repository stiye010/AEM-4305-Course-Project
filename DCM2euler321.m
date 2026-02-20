function [euler321] = DCM2euler321(DCM)
% DCM2euler321 takes in a DCM and outputs a 321 Euler Angle set

euler1 = atan(DCM(1,2)/DCM(1,1));
euler2 = -asin(DCM(1,3));
euler3 = atan(DCM(2,3)/DCM(3,3));

euler321 = [euler1;euler2;euler3];
end