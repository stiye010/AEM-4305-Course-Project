function [C_TRIAD] = DCMEstimate(rwc_n,rsc_n,rwc_b,rsc_b)
% DSMEstimate takes in the measured position vectors from genMeasurements.m
% and uses the TRIAD.m function to estimate the DCM

N = size(rwc_n,1);

C_TRIAD = cell(N,1);

for i=1:N
    C_TRIAD{i} = TRIAD(rsc_n(i,:)',rwc_n(i,:)',rsc_b(i,:)',rwc_b(i,:)');
end
end