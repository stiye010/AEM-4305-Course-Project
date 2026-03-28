% mainSim.m
% numerically integrates a spacecrafts tajectory and then plots the
% corresponding trajectories in position space

%% Project Part 2
close all; clear; clc

% Load in Paramaters and Initial Conditions
params = loadParameters();

% Simulate
[t1,x1] = ode45(@(t,x) f2BP(x,params), ...
    params.tspan, params.x0, params.odeOptions1);

[t2,x2] = ode45(@(t,x) f2BP(x,params), ...
    params.tspan, params.x0, params.odeOptions2);

[t3,x3] = ode45(@(t,x) f2BP(x,params), ...
    params.tspan, params.x0, params.odeOptions3);

% Plot trajectory
figure
plot3(x1(:,1),x1(:,2),x1(:,3),'r')
hold on
plot3(x2(:,1),x2(:,2),x2(:,3),'b')
plot3(x3(:,1),x3(:,2),x3(:,3),'k')
grid on
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
legend('Tol = 1e-2','Tol = 1e-6','Tol = 1e-12');

% Energy
E1 = computeScEnergy(x1,params);
E2 = computeScEnergy(x2,params);
E3 = computeScEnergy(x3,params);

% Energy Plots
% All Energies
figure
plot(t1, E1 - E1(1),'r')
hold on
plot(t2, E2 - E2(1),'b')
plot(t3, E3 - E3(1),'k')
grid on
xlabel('Time [s]')
ylabel('E(t) - E(0)')
legend('Tol = 1e-2','Tol = 1e-6','Tol = 1e-12',"Location",'southwest')

% Tolerance 1e-6 Energy
figure
plot(t2, E2 - E2(1),'r')
grid on
xlabel('Time [s]')
ylabel('E(t) - E(0)')
legend('RelTol=1e-6')

% Tolerance 1e-12 Energy
figure
plot(t3, E3 - E3(1),'b')
grid on
xlabel('Time [s]')
ylabel('E(t) - E(0)')
legend('Tol = 1e-12')


%% Part 3 of Project
rn = x3(:,1:3);
vn = x3(:,4:6);

rd = posN2Rd(rn,vn,params);

figure
subplot(3,1,1)
plot(t3,rd(:,1),'b',"LineWidth",1.2);
grid on; ylabel('r_1d (km)');
subplot(3,1,2)
plot(t3,rd(:,2),'r',"LineWidth",1.2);
grid on; ylabel('r_2d (km)');
subplot(3,1,3)
plot(t3,rd(:,3),'g',"LineWidth",1.2);
grid on; ylabel('r_3d (km)'); xlabel('Time(s)');
