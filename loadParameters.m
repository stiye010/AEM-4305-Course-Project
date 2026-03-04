function params = loadParameters()
% loadParameters takes in no inputs but outputs a stuct containing:
%   mu_moon,mass of spacecraft, simulation time span, simulation initial
%   conditions

% Physical Parameters
params.mu_moon = 4902.800; % Gravitational constant of the moon (km^3/s^2)
params.ms = 4;     % Mass of spacecraft in (kg)

% Simulation Parameters
tstart = 0;
tend = 12000; % in seconds
params.tspan = [tstart,tend];   % Time span for the simulation

params.odeOptions1 = odeset('RelTol', 1e-2,'AbsTol',1e-2); % ode options
params.odeOptions2 = odeset('RelTol', 1e-6,'AbsTol',1e-6);
params.odeOptions3 = odeset('RelTol', 1e-12,'AbsTol',1e-12);

% Initial Conditions
r0 = [0; 1343.503; 1343.503];  % initial position (km)
v0 = [0; -1.135874; 1.135874]; % initial velocity (km/s)
params.x0 = [r0; v0];      % Initial condtitios for the simulation

end