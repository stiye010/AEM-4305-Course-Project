function [rwc_n,rsc_n,rwc_b,rsc_b] = genMeasurements(x)
% genMeasurements take in the state vectors from the MainSim.m and outputs
% the the measurements from the sensors

N = size(x,1);

rwc_n = zeros(N,3);
rsc_n = zeros(N,3);
rwc_b = zeros(N,3);
rsc_b = zeros(N,3);

rsw_n = [0; 150000000; 0];


for i = 1:N
    % Extract state
    rwc_n_i = x(i,1:3)'; 
    q_bn = x(i,7:10)';     
    
    % Convert quaternion to DCM
    C_bn = quaternion2DCM(q_bn);

    % Get sun to s/c vector
    rsc_n_i = rsw_n + rwc_n_i;

    % Generate noisy measurements 
    rsc_b_i = sunSensorNoisy(rsc_n_i,C_bn);
    rwc_b_i = lunarSensorNoisy(rwc_n_i,C_bn);

    % Store values
    rwc_n(i,:) = rwc_n_i';
    rsc_n(i,:) = rsc_n_i';
    rwc_b(i,:) = rwc_b_i';
    rsc_b(i,:) = rsc_b_i';
end
end