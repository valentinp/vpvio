%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('helpers');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('kitti/devkit');
addpath('kitti');
addpath('/home/valentin/Dropbox/Research/Ubuntu/opengv/matlab');
addpath('~/mexopencv/');
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');

%% Parameters
rosBagFileName = '/home/valentin/Desktop/Crucifix/2014-06-25-20-42-24.bag';
imuTopic = '/microstrain/imu/data';
flea3Topic = '/flea3/camera/image_rect';


imageSize = [1280 960];
%Intrinsics
K = [1233.234863 0.000000 668.973851;
0.000000 1251.191040 490.969672;
0.000000 0.000000 1.000000];
R_camimu = rotyd(-180)*rotxd(90);
p_camimu_imu = [-3; 1; -2]*0.01;
T_camimu = [R_camimu -R_camimu*p_camimu_imu; 0 0 0 1];
%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
bag = ros.Bag.load(rosBagFileName);
bag.info()

%% Read all messages 
bagImageData = bag.readAll({flea3Topic});
bagImuData = bag.readAll({imuTopic});
fprintf('Read %i images\n', length(bagImageData));
fprintf('Read %i IMU data points\n', length(bagImuData));

monoImageData = {};
imuData = {};

%% Process images 
monoImageData.rectImages =zeros(imageSize(2), imageSize(1), length(bagImageData));
monoImageData.timestamps = zeros(1, length(bagImageData));
for i=1:length(bagImageData)
    monoImageData.rectImages(:,:,i) = reshape(bagImageData{i}.data, imageSize(1), imageSize(2))';
    monoImageData.timestamps(i) = bagImageData{i}.header.stamp.time;
end
%clear bagImageData;

%% Process imu data
% imuData: struct with IMU data:
%           imuData.timestamps: 1xN 
%           imuData.measAccel: 3xN
%           imuData.measOmega: 3xN
%           imuData.measOrient: 4xN (quaternion q_sw, with scalar in the
%           1st position. The world frame is defined as the N-E-Down ref.
%           frame.
% monoImageData:
%           monoImageData.timestamps: 1xM
%           monoImageData.rectImages: WxHxM


imuData.timestamps = zeros(1, length(bagImuData));
imuData.measAccel = zeros(3, length(bagImuData));
imuData.measOrient = zeros(4, length(bagImuData));
imuData.measOmega = zeros(3, length(bagImuData));
for i=1:length(bagImuData)
     imuData.timestamps(1,i) = bagImuData{i}.header.stamp.time;
     imuData.measAccel(:,i) = bagImuData{i}.linear_acceleration;
     imuData.measOrient(:,i) = [bagImuData{i}.orientation(4); bagImuData{i}.orientation(1:3)];
     imuData.measOmega(:,i) = bagImuData{i}.angular_velocity;
end

%% VIO pipeline
%Set parameters
close all;
simParams = {};
simParams.INIT_DISPARITY_THRESHOLD = 10^2; %in squared pixels
simParams.KF_DISPARITY_THRESHOLD = 2^2; %in squared pixels
simParams.MIN_FEATURE_MATCHES = 3;
simParams.LIVE_PLOT = false;
simParams.VERBOSE = true;

xInit.p = zeros(3,1);
xInit.v = zeros(3,1);
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1;zeros(3,1)];

g_w = -1*rotmat_from_quat(imuData.measOrient(:,1))'*[0 0 -9.81]';
noiseParams.sigma_g = 0;
noiseParams.sigma_a = 0;
noiseParams.sigma_bg = 0;
noiseParams.sigma_ba = 0;
noiseParams.tau = 10^12;

 
%The pipeline
[T_wc_estimated,T_wimu_estimated, keyFrames] = VIOPipelineV2(K, T_camimu, monoImageData, imuData, simParams, noiseParams, xInit, g_w);

% Plot the result
figure;
p_CAMw_w_est = zeros(3, size(T_wc_estimated,3));

for i = 1:size(T_wc_estimated,3)
    p_CAMw_w_est(:,i) = homo2cart(T_wc_estimated(:,:,i)*[0;0;0;1]);
end

plot3(p_CAMw_w_est(1,:), p_CAMw_w_est(2,:), p_CAMw_w_est(3,:),'.-r');
hold on; grid on;
view([0 90]);
%% G2O
% Extract unique landmarks
landmarks.id = [];
landmarks.position = [];
landmarks.count = [];

keyFrameIds = zeros(1, length(keyFrames));
totalLandmarks = 0;

for i = 1:length(keyFrames)
     kf = keyFrames(i);
     
    landmarkIds = kf.landmarkIds;
    landmarkPositions = kf.pointCloud;
    totalLandmarks = totalLandmarks + size(landmarkPositions, 2);
    
    %Deprecated version
    [newLandmarkIds,idx] = setdiff(landmarkIds,landmarks.id);
    landmarks.id = [landmarks.id newLandmarkIds];
    landmarks.position = [landmarks.position landmarkPositions(:, idx)];

    %New version: landmarks initialized to mean position
%     for j = 1:length(landmarkIds)
%         lid = landmarkIds(j);
%         if ismember(lid, landmarks.id)
%           landmarks.position(:,landmarks.id == lid) = landmarks.position(:,landmarks.id == lid) + landmarkPositions(:,j);
%           landmarks.count(landmarks.id == lid) = landmarks.count(landmarks.id == lid) + 1;
%         else
%           landmarks.id = [landmarks.id lid];
%           landmarks.position = [landmarks.position landmarkPositions(:, j)];
%           landmarks.count = [landmarks.count 1];
%         end
%     end
    
    keyFrameIds(i) = kf.imuMeasId;
end

%landmarks.position = landmarks.position./repmat(landmarks.count, [3,1]);

disp(['Total Keyframes: ' num2str(length(keyFrames))]);
disp(['Total unique tracked landmarks: ' num2str(length(landmarks.id))]);
disp(['Total triangulated landmarks: ' num2str(totalLandmarks)]);

%
close all
visualizeVO([], T_wc_estimated(:,:,keyFrameIds), landmarks.position, '- Non Optimized')
%figure
%plot3(landmarks.position(1,:),landmarks.position(2,:),landmarks.position(3,:), '*b')
%visualizeVO([], keyFrames(1).T_wk, keyFrames(1).pointCloud, '- Non Optimized')

%%
%close all;
%%
%Use GTSAM?
% import gtsam.*;
% addpath('/home/valentin/Dropbox/Research/Ubuntu/gtsam_toolbox/');
% [T_wc_list_opt, landmarks_w_opt] = processWithGTSAM(keyFrames,landmarks, K);


% Optimize the result
if exist('keyframes.g2o', 'file') == 2
delete('keyframes.g2o');
end
if exist('opt_keyframes.g2o', 'file') == 2
delete('opt_keyframes.g2o');
end



close all;
visualizeVO([], T_wc_estimated(:,:,keyFrameIds), landmarks.position, '- Non Optimized')

exportG2ODataExpMap(keyFrames,landmarks, K, 'keyframes.g2o', false)

%command = '!g2o_bin/g2o -i 1000  -  v -robustKernel DCS -solver   lm_dense6_3 -o opt_keyframes.g2o test.g2o';
%command = '!g2o_bin/g2o -i 25 -v -solver lm_var -solverProperties initialLambda=0.001 -o -printSolverProperties opt_keyframes.g2o test.g2o';
%-robustKernel Cauchy -robustKernelWidth 1

command = '!g2o_bin/g2o -i 1000 -v -solver lm_dense6_3 -o  opt_keyframes.g2o keyframes.g2o';
eval(command);
%command2 = '!g2o_bin/g2o -i 1000 -v  -solver lm_dense6_3 -solverProperties initialLambda=0.001 -o  opt_keyframes.g2o int_keyframes.g2o';
%eval(command2);

[T_wc_list_opt, landmarks_w_opt] = importG2ODataExpMap('opt_keyframes.g2o');

visualizeVO([], T_wc_list_opt, landmarks_w_opt, '- Optimized')

% Plot the result
figure;
p_CAMw_w_estOpt = zeros(3, size(T_wc_list_opt,3));
p_CAMw_w_est = zeros(3, size(T_wc_estimated,3));

for i = 1:size(p_CAMw_w_est,2)
    p_CAMw_w_est(:,i) = homo2cart(T_wc_estimated(:,:,i)*[0;0;0;1]);
end
for i = 1:size(p_CAMw_w_estOpt, 2)
    p_CAMw_w_estOpt(:,i) = homo2cart(T_wc_list_opt(:,:,i)*[0;0;0;1]);
end

p_CAMw_w_est = p_CAMw_w_est(:, keyFrameIds);


%  plot(p_CAMw_w_GT(1,:), '-r');
%  hold on; grid on;
%  plot(p_CAMw_w_GT(2,:), '-g');
%  plot(p_CAMw_w_GT(3,:), '-b');
% 
% plot(p_CAMw_w_estOpt(1,:), '--r');
% hold on; grid on;
% plot(p_CAMw_w_estOpt(2,:), '--g');
% plot(p_CAMw_w_estOpt(3,:), '--b');
% 
% plot(p_CAMw_w_est(1,:), '.-r');
% hold on; grid on;
% plot(p_CAMw_w_est(2,:), '.-g');
% plot(p_CAMw_w_est(3,:), '.-b');

hold on; grid on;
plot3(p_CAMw_w_est(1,:), p_CAMw_w_est(2,:), p_CAMw_w_est(3,:),'.-r');
plot3(p_CAMw_w_estOpt(1,:),p_CAMw_w_estOpt(2,:),p_CAMw_w_estOpt(3,:), '.-g');

view([0 90]);

legend('IMU Only','VIO', 4);


