function [Gamma] = GammaQuaternion(q)
%GammaQuaternion takes in a quaternion and outputs Gamma_ba(q)
n = q(1);
E = q(2:4);

Gamma = (1/2)*[-E'; n*eye(3)+Cross(E)];
end