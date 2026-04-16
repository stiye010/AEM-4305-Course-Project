function [xdot] = f2BP(x,params)
% f2BP takes in x and the parameters struct and outputs the EOM xdot

r_sw = x(1:3); % Position
v_sw = x(4:6); % Velocity

r = sqrt(r_sw'*r_sw);

mu = params.mu_moon;

xdot = [v_sw(1); v_sw(2); v_sw(3); ...
        -mu*r_sw(1)/r^3; -mu*r_sw(2)/r^3; -mu*r_sw(3)/r^3];
end