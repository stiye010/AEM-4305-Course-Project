function [q_dot] = quaternionKinematics(w,q)
% quaternionKinematics takes in an angular velocity vector and a quaternion
% and outputs the time derivative of the quternion (q_dot)
% This uses the GammaQuaternion Function

q_dot = GammaQuaternion(q)*w;
end