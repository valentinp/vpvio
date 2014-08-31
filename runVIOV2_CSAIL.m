%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/VIO/helpers');
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/VIO/keyframe_imu');
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB//utils');
addpath('/home/valentin/Dropbox/Research/Ubuntu/opengv/matlab');
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');
if ismac
    addpath('/Users/valentinp/Research/gtsam_toolbox');
else
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
end
 import gtsam.*;

%% Parameters
rosBagFileName = '2012-01-27-07-37-01.bag';
imuTopic = '/torso_lift_imu/data';
flea3Topic = '/wide_stereo/left/image_raw';


imageSize = [640 480];
%Intrinsics
K = [424.782760, 0.000000, 323.159380;
0.000000,  424.990290, 249.008680;
0.000000,  0.000000,   1.000000];
R_camimu = rotyd(-180)*rotxd(90);
p_camimu_imu = [-3; 1; -2]*0.01;
T_camimu = [R_camimu -R_camimu*p_camimu_imu; 0 0 0 1];
%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.

bag = ros.Bag.load(rosBagFileName);
bag.info()

%%
tree = ros.TFTree(bag);
transform_params = tree.lookup('wide_stereo_gazebo_l_stereo_camera_optical_frame', 'imu_link', tree.time_begin+1)
%%
T_camimu = [rotmat_from_quat([transform_params.rotation(4); transform_params.rotation(1:3)]) transform_params.translation; 0 0 0 1];


%% Read all messages 
bagImageData = bag.readAll({flea3Topic});
bagImuData = bag.readAll({imuTopic});

fprintf('Read %i images\n', length(bagImageData));
fprintf('Read %i IMU data points\n', length(bagImuData));

monoImageData = {};
imuData = {};

%% Process images and IMU data

%Only use the first N seconds:
useFirstNSeconds = 60;

%CAMERA
%Initialize
monoImageData.rectImages = zeros(imageSize(2), imageSize(1));
monoImageData.timestamps = 0;

currDeltaTime = 0;
firstTimeStamp = bagImageData{1}.header.stamp.time;
i = 1;
while currDeltaTime < useFirstNSeconds
    monoImageData.rectImages(:,:,i) = reshape(bagImageData{i}.data, imageSize(1), imageSize(2))';
    timestamp = bagImageData{i}.header.stamp.time;
    monoImageData.timestamps(i) = timestamp;
    currDeltaTime = timestamp - firstTimeStamp;
    i = i+1;
end


%IMU
imuData.timestamps = zeros(1, 1);
imuData.measAccel = zeros(3, 1);
imuData.measOrient = zeros(4, 1);
imuData.measOmega = zeros(3, 1);
currDeltaTime = 0;
firstTimeStamp = bagImageData{1}.header.stamp.time;
i = 1;
while currDeltaTime < useFirstNSeconds
     timestamp = bagImuData{i}.header.stamp.time;
     imuData.timestamps(1,i) = timestamp;
     imuData.measAccel(:,i) = bagImuData{i}.linear_acceleration;
     imuData.measOrient(:,i) = [bagImuData{i}.orientation(4); bagImuData{i}.orientation(1:3)];
     imuData.measOmega(:,i) = bagImuData{i}.angular_velocity;
     currDeltaTime = timestamp - firstTimeStamp;
     i = i+1;
end


clear bagImageData;
clear bagImuData;
%% VIO pipeline
%Set parameters
if ismac
    addpath('/Users/valentinp/Research/opengv/matlab');
    addpath('/Users/valentinp/Research/mexopencv');
else
    addpath('~/Dropbox/Research/Ubuntu/opengv/matlab');
    addpath('~/mexopencv/');
end


xInit.p = zeros(3,1);
xInit.v = zeros(3,1);
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1;zeros(3,1)];

g_w = -1*rotmat_from_quat(imuData.measOrient(:,1))'*[0 0 9.81]';
noiseParams.sigma_g = 1e-3;
noiseParams.sigma_a = 1e-3;
noiseParams.sigma_bg = 1e-3;
noiseParams.sigma_ba = 1e-3;
noiseParams.tau = 10^12;

 
%Set parameters
close all;
pipelineOptions.featureCount = 1000;
pipelineOptions.initDisparityThreshold = 3;
pipelineOptions.kfDisparityThreshold = 1;
pipelineOptions.showFeatureTracks = true;


pipelineOptions.inlierThreshold = 0.5^2;
pipelineOptions.inlierMinDisparity = 3;
pipelineOptions.inlierMaxForwardDistance = 100;

pipelineOptions.verbose = true;

%The pipeline
[T_wc_estimated,T_wimu_estimated, T_wimu_gtsam, keyFrames] = VIOPipelineV2_GTSAM(K, T_camimu, monoImageData, imuData, pipelineOptions, noiseParams, xInit, g_w);



% Plot the estimated values
figure;
p_IMUw_w_GT = zeros(3, length(keyFrames));
p_IMUw_w_int = zeros(3, length(keyFrames));
p_IMUw_w_gtsam = zeros(3, length(keyFrames));

% Get the keyframe IMU ids
keyFrameIdsIMU = zeros(1, length(keyFrames));
for kf_i = 1:length(keyFrames)
    keyFrameIdsIMU(kf_i) = keyFrames(kf_i).imuMeasId;
    p_IMUw_w_GT(:,kf_i) = homo2cart(T_wIMU_GT(:,:,keyFrames(kf_i).imuMeasId)*[0;0;0;1]);
    p_IMUw_w_int(:,kf_i) = homo2cart(T_wimu_estimated(:,:,keyFrames(kf_i).imuMeasId)*[0;0;0;1]);
    p_IMUw_w_gtsam(:,kf_i) = homo2cart(T_wimu_gtsam(:,:,kf_i)*[0;0;0;1]);
end



plot3(p_IMUw_w_GT(1,:),p_IMUw_w_GT(2,:),p_IMUw_w_GT(3,:), '.-k');
hold on; grid on;
plot3(p_IMUw_w_int(1,:), p_IMUw_w_int(2,:), p_IMUw_w_int(3,:),'.-r');
plot3(p_IMUw_w_gtsam(1,:),p_IMUw_w_gtsam(2,:),p_IMUw_w_gtsam(3,:), '.-g');
set (gcf(), 'outerposition', [25 1000, 560, 470])

view([0 90]);

legend('Ground Truth', 'Integrated','GTSAM', 4)


% Calculate Relative Pose Error
% Take only the poses at the keyframes
T_wIMU_GT_sync = T_wIMU_GT(:,:,keyFrameIdsIMU);
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


