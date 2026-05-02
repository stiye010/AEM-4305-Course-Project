function rgwn = buildGatewayPosInterp()
% =========================================
% =========================================
%
% Build Gateway Position Interpolant
% By: Damennick Henry
% Date: 4/16/26
% Description: Function to create interpolant of Gateway's position
% interpolation
%
%   MAKE SURE TO HAVE THE FILE gatewayPosTimeHistory.mat IN THE SAME FOLDER AS
%   YOUR MAIN FUNCTION
%
% =========================================
% =========================================

% Load position, time history, and orbit period
data = load('gatewayPosTimeHistory.mat');
r = data.r; t = data.t; T = data.T;

% Create interpolation
rx = griddedInterpolant(t, r(1,:), 'spline');
ry = griddedInterpolant(t, r(2,:), 'spline');
rz = griddedInterpolant(t, r(3,:), 'spline');
% Wrap them into a single anonymous function
rgwn = @(t) [rx(mod(t, T)); ...
          ry(mod(t, T)); ...
          rz(mod(t, T))];