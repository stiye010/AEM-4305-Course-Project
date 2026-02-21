function [S] = SQuaternion(q)
%SQuaternion takes in a quaternion and outputs S_ba(q)
n = q(1);
E = q(2:4);

S = [-2*E, 2*(n*eye(3)-Cross(E))];
end