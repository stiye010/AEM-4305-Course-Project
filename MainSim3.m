%% Project Part 4
% mainSim.m
% numerically integrates a spacecrafts orbital and attitude motion 
% full 6 degree-of-freedom simulation

close all; clear; clc

% Load in Paramaters and Initial Conditions
params = loadParameters();

% Simulate
[t,x] = ode45(@(t,x) f2BPGravGrad(x), ...
    params.tspan, params.x0, params.odeOptions3);

r = x(:,1:3);
v = x(:,4:6);
n = x(:,7);
e = x(:,8:10);
q = x(:,7:10);
w = x(:,11:13);

% Quaternion norm error
q_norm = sum(q.^2,2);
q_err = q_norm - 1;

% Energy
T = computeScRotEnergy(w,params);
T_diff = T - T(1);

% Euler Angles
eul = quats2EulAng321(q);

% Plots