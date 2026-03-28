function [w_RdN] = wRdN(rn,params)
% wRdN takes in the position in the inertial frame and the paramaters 
% struct and outputs the angular velocity of Frd relative to Fn resolved in
% Frd
mu = params.mu_moon;
wn = sqrt(mu/norm(rn)^2);
w_RdN = [0; 0; wn];
end