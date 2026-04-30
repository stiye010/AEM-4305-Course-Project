function [Cbn] = TRIAD(V_n1,V_n2,V_b1,V_b2)
% This function performs the TRIAD algorithm
%
% It takes in 2 (non-normalized) vector measurments resolved in the 
% inertial frame and 2 (non-normalized)vector measurments resolved in the
% body frame as inputs
%
% This function provides an estimate of Cbn as an output

w1_n = V_n1/norm(V_n1);
w2_n = Cross(w1_n)*V_n2 / norm(Cross(w1_n)*V_n2);
w3_n = Cross(w1_n)*w2_n;

Cnw = [w1_n, w2_n, w3_n];

w1_b = V_b1/norm(V_b1);
w2_b = Cross(w1_b)*V_b2 / norm(Cross(w1_b)*V_b2);
w3_b = Cross(w1_b)*w2_b;

Cbw = [w1_b, w2_b, w3_b];

Cbn = Cbw * Cnw';
end