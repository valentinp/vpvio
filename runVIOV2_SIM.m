%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('helpers');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('simulation');
if ismac
    addpath('/Users/valentinp/Research/opengv/matlab');
else
    addpath('~/Dropbox/Research/Ubuntu/opengv/matlab');
end
if ismac
    addpath('/Users/valentinp/Research/gtsam_toolbox');
else
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
end
 import gtsam.*;
%% Random number seed
%rng(12341312);



%% Specify transformation
T_camimu = eye(4);



%% Generate the measurments
%Generate the trajectory motion
close all;
disp('Generating trajectory...');
%Simulation parameters
simSetup.imuRate = 200; % Hz
simSetup.cameraRate = 5; % Hz
simSetup.cameraResolution = [640, 480]; %pixels
simSetup.simTime =  38;    % seconds

simSetup.gyroNoiseStd = 1e-3; 
simSetup.accelNoiseStd = 1e-3;
simSetup.gyroBiasStd = 3e-3; 
simSetup.accelBiasStd = 8e-5;
simSetup.pixelNoiseStd = 0.25; %pixels



[T_wImu_GT, imuData] = genTrajectoryCircle(simSetup);

T_wCam_GT = T_wImu_GT;
for i = 1:size(T_wImu_GT, 3)
    T_wCam_GT(:,:,i) = T_wImu_GT(:,:,i)*inv(T_camimu);
end

%Generate the true landmarks
landmarks_w = genLandmarks([-20,20], [-20,20],[0,10], 2000);

%Set the camera intrinsics
focalLength = 4*1e-3; %10 mm
pixelWidth = 4.8*1e-6;
% 
K  = [focalLength/pixelWidth 0 640;
    0 focalLength/pixelWidth 512;
    0 0 1];


%Generate image data
disp('Generating image measurements...');

imageMeasurements = genImageMeasurements(T_wCam_GT, landmarks_w, K, simSetup);
%
%visualizeVO([], T_wCam_GT(:,:,1:10:size(T_wCam_GT,3)), zeros(3,1), 'Simulation')

disp('Done generating measurements.');


%% VIO pipeline

%Set parameters
close all;
pipelineOptions.initDisparityThreshold = 1;
pipelineOptions.kfDisparityThreshold = 100;
pipelineOptions.showFeatureTracks = true;


pipelineOptions.inlierThreshold = 0.5^2;
pipelineOptions.inlierMinDisparity = 10;
pipelineOptions.inlierMaxForwardDistance = 100;
pipelineOptions.verbose = false;

xInit.p = imuData.initialPosition;
xInit.v = imuData.initialVelocity;
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = imuData.measOrient(:,1);

g_w = [0 0 -9.81]';

noiseParams.sigma_g = 1e-3;
noiseParams.sigma_a = 1e-3;
noiseParams.sigma_bg = 1e-10;
noiseParams.sigma_ba = 1e-10;
noiseParams.tau = 10^12;
        
                  
%The pipeline
[T_wc_estimated,T_wimu_estimated, T_wimu_gtsam, keyFrames] = VIOPipelineV2_SIM(K, T_camimu, imageMeasurements, imuData, pipelineOptions, noiseParams, xInit, g_w, landmarks_w);


% Plot the estimated values
figure;
set (gcf(), 'outerposition', [25 800, 560, 470])
p_IMUw_w_GT = zeros(3, length(keyFrames));
p_IMUw_w_int = zeros(3, length(keyFrames));
p_IMUw_w_gtsam = zeros(3, length(keyFrames));

% Get the keyframe IMU ids
keyFrameIdsIMU = zeros(1, length(keyFrames));
for kf_i = 1:length(keyFrames)
    keyFrameIdsIMU(kf_i) = keyFrames(kf_i).imuMeasId;
    p_IMUw_w_GT(:,kf_i) = homo2cart(T_wImu_GT(:,:,keyFrames(kf_i).imuMeasId)*[0;0;0;1]);
    p_IMUw_w_int(:,kf_i) = homo2cart(T_wimu_estimated(:,:,keyFrames(kf_i).imuMeasId)*[0;0;0;1]);
    p_IMUw_w_gtsam(:,kf_i) = homo2cart(T_wimu_gtsam(:,:,kf_i)*[0;0;0;1]);
end

plot3(p_IMUw_w_GT(1,:),p_IMUw_w_GT(2,:),p_IMUw_w_GT(3,:), '.-k');
hold on; grid on;
plot3(p_IMUw_w_int(1,:), p_IMUw_w_int(2,:), p_IMUw_w_int(3,:),'.-r');
plot3(p_IMUw_w_gtsam(1,:),p_IMUw_w_gtsam(2,:),p_IMUw_w_gtsam(3,:), '.-g');

view([0 90]);

legend('Ground Truth', 'Integrated','GTSAM', 4)


% Calculate Relative Pose Error
% Take only the poses at the keyframes
T_wIMU_GT_sync = T_wImu_GT(:,:,keyFrameIdsIMU);
T_wimu_est_sync = T_wimu_estimated(:,:, keyFrameIdsIMU);

RMSE_RPE_imuonly_list = zeros(1, size(T_wIMU_GT_sync,3)-1);
RMSE_RPE_gtsam_list =zeros(1, size(T_wIMU_GT_sync,3)-1);

%Iterative through different step sizes
for dstep = 1:size(T_wIMU_GT_sync,3)-1

    RPE_opt =  zeros(4,4, size(T_wIMU_GT_sync,3) - dstep);
    RPE_imuonly = RPE_opt;

    for i = 1:(size(T_wIMU_GT_sync,3)-dstep)
        RPE_opt(:,:,i) = inv(inv(T_wIMU_GT_sync(:,:,i))*T_wIMU_GT_sync(:,:,i+dstep))*inv(T_wimu_gtsam(:,:,i))*T_wimu_gtsam(:,:,i+dstep); 
        RPE_imuonly(:,:,i) = inv(inv(T_wIMU_GT_sync(:,:,i))*T_wIMU_GT_sync(:,:,i+dstep))*inv(T_wimu_est_sync(:,:,i))*T_wimu_est_sync(:,:,i+dstep);  
    end

    %Calculate the root mean squared error of all the relative pose errors
    RMSE_RPE_opt = 0;
    RMSE_RPE_imuonly = 0;

    for i = 1:size(RPE_opt,3)
        RMSE_RPE_opt = RMSE_RPE_opt + norm(RPE_opt(1:3,4,i))^2;
        RMSE_RPE_imuonly = RMSE_RPE_imuonly + norm(RPE_imuonly(1:3,4,i))^2;  
    end
    RMSE_RPE_imuonly_list(dstep) = sqrt(RMSE_RPE_imuonly/size(RPE_opt,3));
    RMSE_RPE_gtsam_list(dstep) = sqrt(RMSE_RPE_opt/size(RPE_opt,3));
end

RMSE_RPE_opt = mean(RMSE_RPE_gtsam_list);
RMSE_RPE_imuonly = mean(RMSE_RPE_imuonly_list);

%Add to the title
title(sprintf('Mean RMSE RPE (Optimized/IMU Only): %.5f / %.5f ', RMSE_RPE_opt, RMSE_RPE_imuonly));

printf('--------- \n End Euclidian Error (Opt/IMU): %.5f / %.5f', norm(p_IMUw_w_GT(:,end) -  p_IMUw_w_gtsam(:, end)) ,norm(p_IMUw_w_GT(:,end) -  p_IMUw_w_int(:, end)));

% Display mean errors
opt_errors = p_IMUw_w_GT -  p_IMUw_w_gtsam;
imu_errors = p_IMUw_w_GT -  p_IMUw_w_int;

mean_opt_euc = mean(sqrt(sum(opt_errors.^2, 1)));
mean_imu_euc = mean(sqrt(sum(imu_errors.^2, 1)));

printf('--------- \n Mean Euclidian Error (Opt/IMU): %.5f / %.5f',mean_opt_euc , mean_imu_euc);
