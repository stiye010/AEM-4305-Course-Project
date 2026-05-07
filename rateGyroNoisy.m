function w_noisy = rateGyroNoisy(w, t)
% =========================================
% =========================================
%
% Rate Gyroscope - Noisy Measurement
% By: Damennick Henry
% Date: 1/2/26
% Description: Compute a noisy measurement of the angular velocity vector
%              using a rate gyroscope model
% Inputs
%       w       - True angular velocity in body frame [3 x 1] (rad/s)
%       t       - Time (s)
% Outputs
%       w_noisy - Noisy angular velocity measurement [3 x 1] (rad/s)
%
% =========================================
% =========================================
% Error magnitude (0.1 degree/s amplitude)
err_rate = 0.1 * pi/180;
% Deterministic sinusoidal noise on each axis
noise_x = err_rate * sin(1*t);
noise_y = err_rate * cos(1.5*t);
noise_z = err_rate * sin(2.0*t);
bias = 1e-2;
% Apply noise to angular velocity measurement
w_noisy = w + [noise_x; noise_y; noise_z] + bias;
end