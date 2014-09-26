%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('helpers');
addpath('triangulation');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('kitti/devkit');
addpath('kitti');


if ismac
    addpath('/Users/valentinp/Research/mexopencv');
    addpath('/Users/valentinp/Research/gtsam_toolbox');
    addpath('~/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-mac64/');
else
    addpath('~/mexopencv/');
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
    addpath('~/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');
end

import gtsam.*;
%% Parameters
rosBagFileName = '~/Desktop/Crucifix/2014-09-23-14-04-30.bag';
imuTopic = '/microstrain/imu/data';
flea3Topic = '/flea3/camera/image_rect';

imageSize = [1280 960];
%Intrinsics
K = [1269.432045 0.000000 634.289798;
0.000000 1267.775848 490.048452;
0.000000 0.000000 1.000000];

T_camimu = [-0.99998256 -0.00105091 -0.00581085 -0.02170681;
 -0.00580698 -0.00365561  0.99997646  0.02019177;
 -0.00107213  0.99999277  0.00364944 -0.04409566;
  0.          0.          0.          1.        ];

%t_imu = t_cam + shift
imuTimeOffset = -0.014776412914409336;
%imuTimeOffset = -0.025;
 
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
startOffset = 150;
bagImageData(1:startOffset) = [];

%% Process images 
monoImageData.rectImages = [];
monoImageData.timestamps = zeros(1, length(bagImageData));
for i=1:length(bagImageData)
%    monoImageData.rectImages(:,:,i) = reshape(bagImageData{i}.data, imageSize(1), imageSize(2))';
    monoImageData.timestamps(i) = bagImageData{i}.header.stamp.time + imuTimeOffset;
end

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


imuData.timestamps = [];
imuData.measAccel = [];
imuData.measOrient = [];
imuData.measOmega = [];
i = 1;
initBiasGyroList = [];
initBiasAccelList = [];
for t=1:length(bagImuData)
        if (bagImuData{t}.header.stamp.time > monoImageData.timestamps(1) && bagImuData{t}.header.stamp.time < monoImageData.timestamps(end))
         imuData.timestamps(1,i) = bagImuData{t}.header.stamp.time;
         imuData.measOrient(:,i) = [bagImuData{t}.orientation(4); bagImuData{t}.orientation(1:3)];
          imuData.measAccel(:,i) = bagImuData{t}.linear_acceleration;
         imuData.measOmega(:,i) = bagImuData{t}.angular_velocity;
         i = i +1;
        end
end

%% Calculate BIAS values
% 
% dt = 1/500;
% biasSec = 0.5;
% g_mag = 9.805;
% linearAccelList = zeros(3,biasSec/dt);
% omegaList = zeros(3,biasSec/dt);
% for imu_i = 1:(biasSec/dt)
%        gVec = rotmat_from_quat(imuData.measOrient(:,imu_i))'*[0 0 g_mag]';
%        linearAccel = imuData.measAccel(:, imu_i) + gVec;
%        linearOmega = imuData.measOmega(:, imu_i);
%        linearAccelList(:, imu_i) = linearAccel;
%        omegaList(:, imu_i) = linearOmega;
%        gList(:, imu_i) = gVec;
% end
% % 
%  omegaBias = mean(omegaList,2)
%  accelBias = mean(linearAccelList,2)
%  g_w = mean(gList, 2)

g_mag = 9.805;
[omegaBias, accelBias, g_w] = loadBiasesFromIMUBag('/home/valentin/Desktop/Crucifix/2014-09-23-14-04-30.bag', g_mag, 11)


%% VIO pipeline
%Set parameters
close all;
xInit.p = zeros(3,1);
xInit.v = zeros(3,1);
xInit.b_g = omegaBias;
xInit.b_a = accelBias;
xInit.q = [1; zeros(3,1)];

%Pipeline
pipelineOptions.featureCount = 50;
pipelineOptions.initDisparityThreshold = 5;
pipelineOptions.kfDisparityThreshold = 5;
pipelineOptions.showFeatureTracks = false;
%pipelineOptions.inlierThreshold = 100^2;
%pipelineOptions.inlierMinDisparity = 1;
pipelineOptions.inlierMaxForwardDistance = 1;
pipelineOptions.verbose = false;


%GTSAM
pipelineOptions.minViewingsForLandmark = 3;
pipelineOptions.obsNoiseSigma = 1;
pipelineOptions.useRobustMEst = false;
pipelineOptions.mEstWeight = 5;
pipelineOptions.maxBatchOptimizerError = 5;
pipelineOptions.triangPointSigma = 5;

noiseParams.sigma_g =  0.01*ones(3,1); 
noiseParams.sigma_a =  0.1*ones(3,1);
noiseParams.sigma_bg =  1e-5*ones(3,1);
noiseParams.sigma_ba = 1e-5*ones(3,1);
noiseParams.init_ba = accelBias;
noiseParams.init_bg = omegaBias;

%The pipeline
[T_wc_estimated,T_wimu_estimated, T_wimu_gtsam, keyFrames] = VIOPipelineV2_GTSAMCrucifixThreeView(K, T_camimu, monoImageData,bagImageData, imuData, pipelineOptions, noiseParams, xInit, g_w);


% Plot the estimated values
figure;
%p_IMUw_w_GT = zeros(3, length(keyFrames));
p_IMUw_w_int = zeros(3, length(keyFrames));
p_IMUw_w_gtsam = zeros(3, length(keyFrames));

% Get the keyframe IMU ids
keyFrameIdsIMU = zeros(1, length(keyFrames));
%p_IMUw_w_int = zeros(3,length(keyFrames));
for kf_i = 1:length(keyFrames)
    keyFrameIdsIMU(kf_i) = keyFrames(kf_i).imuMeasId;
    %p_IMUw_w_GT(:,kf_i) = homo2cart(T_wIMU_GT(:,:,keyFrames(kf_i).imuMeasId)*[0;0;0;1]);
    p_IMUw_w_int(:,kf_i) = homo2cart(T_wimu_estimated(:,:,keyFrames(kf_i).imuMeasId)*[0;0;0;1]);
    p_IMUw_w_gtsam(:,kf_i) = homo2cart(T_wimu_gtsam(:,:,kf_i)*[0;0;0;1]);
end



%plot3(p_IMUw_w_GT(1,:),p_IMUw_w_GT(2,:),p_IMUw_w_GT(3,:), '.-k');
hold on; grid on;
plot3(p_IMUw_w_int(1,:), p_IMUw_w_int(2,:), p_IMUw_w_int(3,:),'.-r');
plot3(p_IMUw_w_gtsam(1,:),p_IMUw_w_gtsam(2,:),p_IMUw_w_gtsam(3,:), '.-g');
set (gcf(), 'outerposition', [25 1000, 560, 470])

xlabel('x'); ylabel('y'); zlabel('z');

%view([0 90]);

legend('Integrated','GTSAM', 4)

printf('Final position difference: %.5f/%.5f (IMU/Opt)',norm(p_IMUw_w_int(:,end) - p_IMUw_w_int(:,1)),  norm(p_IMUw_w_gtsam(:,end) - p_IMUw_w_gtsam(:,1)));