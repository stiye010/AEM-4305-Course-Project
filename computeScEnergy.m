function [E] = computeScEnergy(x, params)
% computeScEnergy takes in the state of the spacecraft and outputs the
% energy

mu = params.mu_moon;
ms = params.ms;

% Extract position and velocity
r = x(:,1:3);
v = x(:,4:6);

% Magnitudes
r_norm = vecnorm(r,2,2);
v_norm = vecnorm(v,2,2);

T = 0.5 * ms .* v_norm.^2;

V = -mu * ms ./ r_norm;

% Total energy
E = T + V;
end