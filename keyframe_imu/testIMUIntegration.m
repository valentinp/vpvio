%% Import matlab rosbag
clear rosbag_wrapper;
clear ros.Bag;
clear all;
clc;
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');
cd('/home/valentin/Dropbox/UTIAS - MASc/Code/VIO/keyframe_imu');

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
bag = ros.Bag.load('2014-06-01-21-59-03.bag');
bag.info()


%% Read IMU Data
imuTopic = '/microstrain/imu/data';
imuData = bag.readAll({imuTopic});
fprintf('Read %i IMU data points\n', length(imuData));


%% Perform integration
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB/utils');

%Initialize the streams
a_stream = zeros(3, length(imuData));
omega_stream = zeros(3, length(imuData));
q_stream = zeros(4, length(imuData));
dt_stream = zeros(1, length(imuData));
g_w = [0 0 9.804]';

%x is composed of {p, q, v, b_g, b_a}
x_0.p = zeros(3,1);
x_0.v = zeros(3,1);
x_0.b_g = zeros(3,1);
x_0.b_a = zeros(3,1);
x_0.q = [imuData{1}.orientation(4); imuData{1}.orientation(1); imuData{1}.orientation(2); imuData{1}.orientation(3)];


noNoise = true;
%Set noise parameters
if noNoise
noise_params.sigma_g = 0;
noise_params.sigma_a = 0;
noise_params.sigma_bg = 0;
noise_params.sigma_ba = 0;
noise_params.tau = 10^12;
else
noise_params.sigma_g = 0.2;
noise_params.sigma_a = 0.002;
noise_params.sigma_bg = 0.002;
noise_params.sigma_ba = 0.002;
noise_params.tau = 0.5;
end

       
    

for i = 2:length(imuData)
    omega = [imuData{i}.angular_velocity(1); imuData{i}.angular_velocity(2); imuData{i}.angular_velocity(3)];
    a = [imuData{i}.linear_acceleration(1); imuData{i}.linear_acceleration(2); imuData{i}.linear_acceleration(3)];
    q = [imuData{i}.orientation(4); imuData{i}.orientation(1); imuData{i}.orientation(2); imuData{i}.orientation(3)];
    dt = imuData{i}.header.stamp.time - imuData{i-1}.header.stamp.time;
    a_stream(:, i-1) = a;
    q_stream(:, i-1) = q;
    omega_stream(:, i-1) = omega;
    dt_stream(i-1) = dt;
end

%Rolling mean window
filter_window_size = 1;
a_stream = tsmovavg(a_stream, 's', filter_window_size, 2);
omega_stream = tsmovavg(omega_stream, 's', filter_window_size, 2);

a_stream = a_stream(:, filter_window_size:end);
omega_stream = omega_stream(:, filter_window_size:end);
dt_stream = dt_stream(:, filter_window_size:end);
q_stream = q_stream(:, filter_window_size:end);



[x_stream, P_stream] = integrateIMUStream(a_stream, omega_stream, dt_stream, x_0, noise_params, g_w);

% Plot the position estimates
close all;
len = length(x_stream) - 5;
pPlot= zeros(3, len);
vPlot= zeros(3, len);
aNoGPlot = zeros(3, len);
for i = 1:len
    pPlot(:, i) = x_stream{i}.p;
    vPlot(:, i) = x_stream{i}.v;
    aNoGPlot(:,i) = rotmat_from_quat(q)*a_stream(:,i) + [0; 0;9.806707082711076];
    %aNoGPlot(:,i) = a_stream(:,i);
end
hFig = figure(1);
set(hFig, 'Position', [0 0 1000 500]);
subplot(2,2,1);
plot(pPlot(1,:), '-r');
hold on;
plot(pPlot(2,:), '-g');
plot(pPlot(3,:), '-b');
legend('X', 'Y', 'Z', 2);
title('Position');
grid on;
subplot(2,2,2);
plot(vPlot(1,:), '-r');
grid on;
hold on
plot(vPlot(2,:), '-g');
plot(vPlot(3,:), '-b');
legend('X', 'Y', 'Z', 2);
title('Speed');
subplot(2,2,3);
plot(aNoGPlot(1,:), '-r');
hold on
plot(aNoGPlot(2,:), '-g');
plot(aNoGPlot(3,:), '-b');
grid on;
legend('X', 'Y', 'Z', 3);
title('Accel');
subplot(2,2,4);
plot3(pPlot(1,:),pPlot(2,:),pPlot(3,:));
xlabel('X'); ylabel('Y');zlabel('Z');
grid on;title('Position');


    