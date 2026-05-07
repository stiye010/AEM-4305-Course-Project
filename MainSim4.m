%% Project Part 6
% Main Sim for Questions 1
% numerically integrates a spacecrafts orbital and attitude motion 
% full 6 degree-of-freedom simulation
% With detumbling control law

close all; clear; clc

% Load in Parameters and Initial Conditions
params = loadParameters();

% Simulate
[t,x] = ode45(@(t,x) f2BPGravGradDetumble(x, t, params), ...
    params.tspan, params.x0, params.odeOptions3);

r = x(:,1:3);
v = x(:,4:6);
q = x(:,7:10);
n = x(:,7);
e = x(:,8:10);
w = x(:,11:13);

% Quaternion norm error
q_norm = sum(q.^2,2);
q_err = q_norm - 1;

% Control Torque
N = length(t);
tau_u = zeros(N,3);
I = params.Ib;
mu = params.mu_moon;
k_d = 0.005;
Kd = [k_d 0 0;...
    0 k_d 0;...
    0 0 k_d];

for i = 1:N
    w_i = w(i,:)';
    r_i = r(i,:)';
    q_i = q(i,:)';
    C_bn_i = quaternion2DCM(q_i);

    tau_gg = 3*mu / (r_i'*r_i)^(5/2) * Cross(C_bn_i*r_i) * I*C_bn_i*r_i;
    tau_u(i,:) = -Kd*w_i - tau_gg;
end

%% Plotting
% Eta and Epsilon
figure
subplot(4,1,1)
plot(t,n,"LineWidth",1.2);
grid on;ylabel('\eta');
subplot(4,1,2)
plot(t,e(:,1),"LineWidth",1.2);
grid on; ylabel('\epsilon_1');
subplot(4,1,3)
plot(t,e(:,2),"LineWidth",1.2);
grid on; ylabel('\epsilon_2');
subplot(4,1,4)
plot(t,e(:,3),"LineWidth",1.2);
grid on; ylabel('\epsilon_3'); xlabel('Time (s)');

% Quaternion error
figure
plot(t,q_err,"LineWidth",1.2);
grid on;
xlabel('Time (s)'); ylabel('q^Tq - 1')

% Angular velocity
figure
subplot(3,1,1)
plot(t,w(:,1),"LineWidth",1.2);
grid on; ylabel('\omega_1 (rad/s)');
subplot(3,1,2)
plot(t,w(:,2),"LineWidth",1.2);
grid on; ylabel('\omega_2 (rad/s)');
subplot(3,1,3)
plot(t,w(:,3),"LineWidth",1.2);
grid on; ylabel('\omega_3 (rad/s)'); xlabel('Time (s)');

% Control Torque
figure
subplot(3,1,1)
plot(t,tau_u(:,1),"LineWidth",1.2);
grid on; ylabel('\tau_u_1 (Nm)');
subplot(3,1,2)
plot(t,tau_u(:,2),"LineWidth",1.2);
grid on; ylabel('\tau_u_2 (Nm)');
subplot(3,1,3)
plot(t,tau_u(:,3),"LineWidth",1.2);
grid on; ylabel('\tau_u_3 (Nm)'); xlabel('Time (s)');