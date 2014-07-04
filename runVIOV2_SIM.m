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
addpath('/home/valentin/Dropbox/Research/Ubuntu/opengv/matlab');

%% Random number seed
rng(12);



%% Specify transformation
T_camimu = eye(4);



%% Generate the measurments
%Generate the trajectory motion
close all;
disp('Generating trajectory...');
%Simulation parameters
simSetup.imuRate = 100; % Hz
simSetup.cameraRate = 20; % Hz
simSetup.cameraResolution = [640, 480]; %pixels
simSetup.simTime =  60;    % seconds

simSetup.gyroNoiseStd = 0.0002; 
simSetup.accelNoiseStd = 0.005;
simSetup.pixelNoiseStd = 1; %pixels



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
visualizeVO([], T_wCam_GT(:,:,1:10:size(T_wCam_GT,3)), zeros(3,1), 'Simulation')

disp('Done generating measurements.');


%% VIO pipeline

%Set parameters

pipelineOptions.initDisparityThreshold = 5;
pipelineOptions.kfDisparityThreshold = 15;
pipelineOptions.showFeatureTracks = true;


pipelineOptions.inlierThreshold = 0.1^2;
pipelineOptions.inlierMinDisparity = 3;
pipelineOptions.inlierMaxForwardDistance = 50;

pipelineOptions.verbose = false;



xInit.p = imuData.initialPosition;
xInit.v = imuData.initialVelocity;
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = imuData.measOrient(:,1);

g_w = [0 0 9.81]';
noiseParams.sigma_g = 0;

noiseParams.sigma_a = 0;
noiseParams.sigma_bg = 0;
noiseParams.sigma_ba = 0;
noiseParams.tau = 10^12;

 
%The pipeline
[T_wc_estimated,T_wimu_estimated, keyFrames] = VIOPipelineV2_SIM(K, T_camimu, imageMeasurements, imuData, pipelineOptions, noiseParams, xInit, g_w);

close all
figure
p_CAMw_w_GT = zeros(3, size(T_wCam_GT,3));
p_CAMw_w_est = zeros(3, size(T_wCam_GT,3));
rpy_est = zeros(3, size(T_wCam_GT, 3) - 1);
rpy_GT = zeros(3, size(T_wCam_GT, 3) - 1);

for i = 1:size(p_CAMw_w_GT,2)
    p_CAMw_w_GT(:,i) = homo2cart(T_wCam_GT(:,:,i)*[0;0;0;1]);
    p_CAMw_w_est(:,i) = homo2cart(T_wc_estimated(:,:,i)*[0;0;0;1]);
    rpy_est(:,i) = rpy_from_rotmat(T_wc_estimated(1:3,1:3,i));
    rpy_GT(:,i) = rpy_from_rotmat(T_wCam_GT(1:3,1:3,i));
end
subplot(2,1,1)
plot(p_CAMw_w_est(1,:), '--r');
hold on; grid on;
plot(p_CAMw_w_est(2,:), '--g');
plot(p_CAMw_w_est(3,:), '--b');

 plot(p_CAMw_w_GT(1,:), '-r');
 hold on; grid on;
 plot(p_CAMw_w_GT(2,:), '-g');
 plot(p_CAMw_w_GT(3,:), '-b');
 subplot(2,1,2)
plot(rpy_GT(1,:), '-r');
hold on; grid on;
plot(rpy_GT(2,:), '-g');
plot(rpy_GT(3,:), '-b');

plot(rpy_est(1,:), '--r');
hold on; grid on;
plot(rpy_est(2,:), '--g');
plot(rpy_est(3,:), '--b');

legend('True Roll', 'True Pitch', 'True Yaw','Est Roll', 'Est Pitch', 'Est Yaw', 2);

 


%% G2O
% Extract unique landmarks
landmarks.id = [];
landmarks.position = [];

keyFrameIds = zeros(1, length(keyFrames));

for i = 1:length(keyFrames)
     kf = keyFrames(i);
     
    landmarkIds = kf.landmarkIds;
    landmarkPositions = kf.pointCloud;
    [newLandmarkIds,idx] = setdiff(landmarkIds,landmarks.id);
    landmarks.id = [landmarks.id newLandmarkIds];
    landmarks.position = [landmarks.position landmarkPositions(:, idx)];
    keyFrameIds(i) = kf.imuMeasId;
end

disp(['Total Keyframes: ' num2str(length(keyFrames))]);
disp(['Total tracked landmarks: ' num2str(length(landmarks.id))]);

%Prune all landmarks that were only observed once

%
%close all
visualizeVO([], T_wc_estimated(:,:,keyFrameIds), landmarks.position, '- Non Optimized')
%figure
%plot3(landmarks.position(1,:),landmarks.position(2,:),landmarks.position(3,:), '*b')
%visualizeVO([], keyFrames(1).T_wk, keyFrames(1).pointCloud, '- Non Optimized')


%%
% g2o options
g2oOptions.maxPixError = 50;
g2oOptions.fixLandmarks = false;
g2oOptions.fixPoses = false;
g2oOptions.motionEdgeInfoMat = diag([1/simSetup.gyroNoiseStd^2 1/simSetup.gyroNoiseStd^2 1/simSetup.gyroNoiseStd^2 1/simSetup.accelNoiseStd^2 1/simSetup.accelNoiseStd^2 1/simSetup.accelNoiseStd^2]);
g2oOptions.obsEdgeInfoMat = 1*eye(2);

%Use GTSAM?
%  addpath('/home/valentin/Dropbox/Research/Ubuntu/gtsam_toolbox/');
%  [T_wc_list_opt, landmarks_w_opt] = processWithGTSAM(keyFrames,landmarks, K, g2oOptions);

%close all;

Optimize the result
if exist('keyframes.g2o', 'file') == 2
delete('keyframes.g2o');
end
if exist('opt_keyframes.g2o', 'file') == 2
delete('opt_keyframes.g2o');
end

exportG2ODataExpMap(keyFrames,landmarks, K, 'keyframes.g2o',g2oOptions)

%command = '!g2o_bin/g2o -i 1000  -v -robustKernel DCS -solver   lm_dense6_3 -o opt_keyframes.g2o test.g2o';
%command = '!g2o_bin/g2o -i 25 -v -solver lm_var -solverProperties initialLambda=0.001 -o -printSolverProperties opt_keyframes.g2o test.g2o';
%-robustKernel Cauchy -robustKernelWidth 1

command2 = '!g2o_bin/g2o -i 30 -v -solver  lm_var -o  opt_keyframes.g2o keyframes.g2o';

%command2 = '!g2o_bin/simple_optimize -i 200 -o  opt_keyframes.g2o keyframes.g2o';

eval(command2);
% [T_wc_list_opt, landmarks_w_opt] = importG2ODataExpMap('opt_keyframes.g2o');


    % Plot the result
figure;
p_CAMw_w_GT = zeros(3, size(T_wCam_GT,3));
p_CAMw_w_estOpt = zeros(3, size(T_wc_list_opt,3));
p_CAMw_w_est = zeros(3, size(T_wCam_GT,3));

for i = 1:size(p_CAMw_w_GT,2)
    p_CAMw_w_GT(:,i) = homo2cart(T_wCam_GT(:,:,i)*[0;0;0;1]);
    p_CAMw_w_est(:,i) = homo2cart(T_wc_estimated(:,:,i)*[0;0;0;1]);

end
for i = 1:size(p_CAMw_w_estOpt, 2)
    p_CAMw_w_estOpt(:,i) = homo2cart(T_wc_list_opt(:,:,i)*[0;0;0;1]);
end

p_CAMw_w_GT = p_CAMw_w_GT(:, keyFrameIds);
p_CAMw_w_est = p_CAMw_w_est(:, keyFrameIds);

 plot(p_CAMw_w_GT(1,:), '-r');
 hold on; grid on;
 plot(p_CAMw_w_GT(2,:), '-g');
 plot(p_CAMw_w_GT(3,:), '-b');

plot(p_CAMw_w_estOpt(1,:), '--r');
hold on; grid on;
plot(p_CAMw_w_estOpt(2,:), '--g');
plot(p_CAMw_w_estOpt(3,:), '--b');

plot(p_CAMw_w_est(1,:), '.-r');
hold on; grid on;
plot(p_CAMw_w_est(2,:), '.-g');
plot(p_CAMw_w_est(3,:), '.-b');



legend('True X', 'True Y', 'True Z','Est Opt X', 'Est  Opt Y', 'Est Opt Z','Est X', 'Est Y', 'Est Z', 2);

% Calculate Relative Pose Error
% Take only the poses at the keyframes
T_wCam_GT_sync = T_wCam_GT(:,:,keyFrameIds);
T_wc_est_sync = T_wc_estimated(:,:, keyFrameIds);

dstep = 30;

RPE_opt =  zeros(4,4, size(T_wCam_GT_sync,3) - dstep);
RPE_imuonly = RPE_opt;

for i = 1:(size(T_wCam_GT_sync,3)-dstep)
    RPE_opt(:,:,i) = inv(inv(T_wCam_GT_sync(:,:,i))*T_wCam_GT_sync(:,:,i+dstep))*inv(T_wc_list_opt(:,:,i))*T_wc_list_opt(:,:,i+dstep); 
    RPE_imuonly(:,:,i) = inv(inv(T_wCam_GT_sync(:,:,i))*T_wCam_GT_sync(:,:,i+dstep))*inv(T_wc_est_sync(:,:,i))*T_wc_est_sync(:,:,i+dstep);  
end

%Calculate the root mean squared error of all the relative pose errors
RMSE_RPE_opt_list = zeros(1, size(RPE_opt,3));
RMSE_RPE_imuonly_list = zeros(1, size(RPE_opt,3));

for i = 1:size(RPE_opt,3)
    RMSE_RPE_opt_list(i) = norm(homo2cart(RPE_opt(:,4,i)));
    RMSE_RPE_imuonly_list(i) = norm(homo2cart(RPE_imuonly(:,4,i)));  
end

RMSE_RPE_imuonly = sqrt(mean(RMSE_RPE_imuonly_list.^2));
RMSE_RPE_opt = sqrt(mean(RMSE_RPE_opt_list.^2));

%Add to the title
title(sprintf('RMSE RPE (Optimized/IMU Only): %.5f / %.5f ', RMSE_RPE_opt, RMSE_RPE_imuonly));

printf('--------- \n End Euclidian Error (Opt/IMU): %.5f / %.5f', norm(p_CAMw_w_GT(:,end) -  p_CAMw_w_estOpt(:, end)) ,norm(p_CAMw_w_GT(:,end) -  p_CAMw_w_est(:, end)));

% Display mean errors
opt_errors = p_CAMw_w_GT -  p_CAMw_w_estOpt;
imu_errors = p_CAMw_w_GT -  p_CAMw_w_est;

mean_opt_euc = mean(sqrt(sum(opt_errors.^2, 1)));
mean_imu_euc = mean(sqrt(sum(imu_errors.^2, 1)));

printf('--------- \n Mean Euclidian Error (Opt/IMU): %.5f / %.5f',mean_opt_euc , mean_imu_euc);