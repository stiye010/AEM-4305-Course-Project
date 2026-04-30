function vector_noisy = lunarSensorNoisy(vector, Cbn)
% =========================================
% =========================================
%
% Lunar Horizon Sensor - Noisy Measurement
% By: Damennick Henry
% Date: 3/12/26
% Description: Compute a noisy measurement of a vector in the body frame
%              using the lunar horizon sensor model
% Inputs
%       vector       - 3x1 vector resolved in the inertial frame
%       Cbn          - DCM of body frame relative to inertial frame [3x3]
%       t            - Time (s)
% Outputs
%       vector_noisy - 3x1 noisy vector measurement in the body frame
%
% =========================================
% =========================================\
rng(1);
% Error angle magnitude (1 degree standard deviation)
err_angle = 5 * pi/180;
% Randomly generated noise in pitch, roll, and yaw
noise_pitch = err_angle * randn();
noise_roll  = err_angle * randn();
noise_yaw   = err_angle * randn();
% Create DCM
Cerror = euler3212DCM([noise_yaw; noise_pitch; noise_roll]);
% Apply rotation noise and transform to body frame
vector_noisy = Cbn*Cerror*vector;
end