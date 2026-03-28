%% Project Part 3
% mainSim.m
% numerically integrates a spacecrafts attitude under torque-free
% conditions

close all; clear; clc

% Load in Paramaters and Initial Conditions
params = loadParameters();

% Simulate
[t,x] = ode45(@(t,x) fAtt(x), ...
    params.tspan, params.x0, params.odeOptions3);

n = x(:,1);
e = x(:,2:4);
q = x(:,1:4);
w = x(:,5:7);

% Quaternion norm error
q_norm = sum(q.^2,2);
q_err = q_norm - 1;

% Energy
T = computeScRotEnergy(w,params);
T_diff = T - T(1);

% Euler Angles
eul = quats2EulAng321(q);

% Plots
% eta
figure
plot(t,n,"LineWidth",1.2);
grid on;
xlabel('Time (s)');ylabel('\eta')

% epsilon
figure
plot(t,e,"LineWidth",1.2);
grid on;
xlabel('Time (s)'); ylabel('\epsilon')
legend('\epsilon_1','\epsilon_2','\epsilon_3')

% quaternion error
figure
plot(t,q_err,"LineWidth",1.2);
grid on;
xlabel('Time (s)'); ylabel('q^Tq - 1')

% angular velocity
figure
plot(t,w,"LineWidth",1.2);
grid on;
xlabel('Time (s)'); ylabel('\omega (rad/s)')
legend('\omega_1','\omega_2','\omega_3');

% Energy
figure
plot(t,T_diff,"LineWidth",1.2);
grid on;
xlabel('Time (s)'); ylabel('T(t) - T(0)');

% Euler Angles
figure
subplot(3,1,1)
plot(t,rad2deg(eul(:,1)),'b',"LineWidth",1.2);
grid on; ylabel('\psi (deg)');
subplot(3,1,2)
plot(t,rad2deg(eul(:,2)),'r',"LineWidth",1.2);
grid on; ylabel('\theta (deg)');
subplot(3,1,3)
plot(t,rad2deg(eul(:,3)),'g',"LineWidth",1.2);
grid on; ylabel('\phi (deg)'); xlabel('Time(s)');