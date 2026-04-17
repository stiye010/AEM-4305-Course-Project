function [E] = computeSc2BPGGEnergy(x,params)
% computeSc2BPGGEnergy takes in the state vectors from ode45 and computes
% the energy of the spacecraft

r = x(:,1:3);
v = x(:,4:6);
w = x(:,11:13);

m = params.ms;
I = params.Ib;
mu = params.mu_moon;

N = size(r,1);
E = zeros(N,1);

for i = 1:N
   T = 1/2 * m*v(i,:)*v(i,:)' + 1/2 * w(i,:)*I*w(i,:)';
   r_mag = sqrt(r(i,:)*r(i,:)');
   Vg_Bw = -mu*m/r_mag;
   Vgg_Bw = mu/(2*r_mag^3) * (3/r_mag^2 * r(i,:)*I*r(i,:)' - trace(I));
   E(i) = T + Vg_Bw + Vgg_Bw;
end
end