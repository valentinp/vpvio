addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');
addpath('../helpers');
addpath('../../MATLAB/utils');

clc;
clear all;
close all;

% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
rosBagFileName = '2014-09-05-20-43-59.bag';
bag = ros.Bag.load(rosBagFileName);
bag.info()
% Read all messages 
imuTopic = '/microstrain/imu/data';
bagImuData = bag.readAll({imuTopic});
imuData.timestamps = [];
imuData.measAccel = [];
imuData.measOrient = [];
imuData.measOmega = [];
for i=1:length(bagImuData)
         imuData.timestamps(1,i) = bagImuData{i}.header.stamp.time;
         imuData.measAccel(:,i) = bagImuData{i}.linear_acceleration;
         imuData.measOrient(:,i) = [bagImuData{i}.orientation(4); bagImuData{i}.orientation(1:3)];
         imuData.measOmega(:,i) = bagImuData{i}.angular_velocity;
end


g_norm = mean(vecNorms( imuData.measAccel))

measAccel = [];
for i = 1:size(imuData.measOmega, 2)
g_w = rotmat_from_quat(imuData.measOrient(:,i))'*[0 0 g_norm]';
measAccel(:,i) = imuData.measAccel(:,i) + g_w;
end



plot(measAccel(1,:));
title('X-Axis Accel');

figure
plot(measAccel(2,:));
title('Y-Axis Accel');


figure
plot(measAccel(3,:));
title('Z-Axis Accel');


mean_a_bias = [mean(measAccel(1,:));mean(measAccel(2,:));mean(measAccel(3,:));]
mean_g_bias = [mean(imuData.measOmega(1,:));mean(imuData.measOmega(2,:));mean(imuData.measOmega(3,:));]






% 
% %%
% Fs = 100;
% x = imuData.measAccel(1,:);
% N = length(x);
% xdft = fft(x);
% xdft = xdft(1:N/2+1);
% psdx = (1/(Fs*N)).*abs(xdft).^2;
% psdx(2:end-1) = 2*psdx(2:end-1);
% freq = 0:Fs/length(x):Fs/2;
% plot(freq,10*log10(psdx)); grid on;
% title('Periodogram Using FFT');
% xlabel('Frequency (Hz)'); ylabel('Power/Frequency (dB/Hz)');
% 
% %%
% freqData.freq = psdx;
% freqData.rate = 100;
% 
% ad = allan(freqData, [0.01:0.1:1000]);
