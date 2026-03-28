function [T] = computeScRotEnergy(w,params)
% computeScRotEnergy takes in the angular velocity from simulation and the 
% paramaters struct and computes the energy of the spacecraft

I = params.Ib;

N = size(w,1);
T = zeros(N,1);

for i = 1:N
    wi = w(i,:)';
    T(i) = 0.5*wi'*I*wi;
end
end