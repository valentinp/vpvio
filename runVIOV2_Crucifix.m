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
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');

if ismac
    addpath('/Users/valentinp/Research/mexopencv');
    addpath('/Users/valentinp/Research/gtsam_toolbox');
else
    addpath('~/mexopencv/');
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
end

import gtsam.*;
%% Parameters
rosBagFileName = '/home/valentin/Desktop/Crucifix/2014-09-12-15-25-27.bag';
imuTopic = '/microstrain/imu/data';
flea3Topic = '/flea3/camera/image_rect';

imageSize = [1280 960];
%Intrinsics
K = [900.940738 0.000000 624.980453;
0.000000 898.971878 490.705403;
0.000000 0.000000 1.000000];

T_camimu = [-0.99988155  0.01287993 -0.00842556 -0.03879945;
 -0.00847907 -0.00410603  0.99995562  0.02125595;
  0.01284476  0.99990862  0.00421475 -0.04425316;
  0.          0.          0.          1.        ];
 
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
monoImageData.rectImages = [];
monoImageData.timestamps = zeros(1, 500);
for i=1:500%length(bagImageData)
    monoImageData.rectImages(:,:,i) = reshape(bagImageData{i}.data, imageSize(1), imageSize(2))';
    monoImageData.timestamps(i) = bagImageData{i}.header.stamp.time;
    i
end
clear bagImageData;

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
        if (bagImuData{t}.header.stamp.time > monoImageData.timestamps(1))
         imuData.timestamps(1,i) = bagImuData{t}.header.stamp.time;
         imuData.measAccel(:,i) = bagImuData{t}.linear_acceleration;
         imuData.measOrient(:,i) = [bagImuData{t}.orientation(4); bagImuData{t}.orientation(1:3)];
         imuData.measOmega(:,i) = bagImuData{t}.angular_velocity;
         i = i +1;
        else
            initBiasGyroList = [initBiasGyroList bagImuData{t}.angular_velocity];
            measA = rotmat_from_quat([bagImuData{t}.orientation(4); bagImuData{t}.orientation(1:3)])'*[0 0 9.8065]' +  bagImuData{t}.linear_acceleration;
            initBiasAccelList = [initBiasAccelList measA];
         end
end

%initBiasGyro = [mean(initBiasGyroList(1,:)); mean(initBiasGyroList(2,:)); mean(initBiasGyroList(3,:)); ]
%initBiasAccel = [mean(initBiasAccelList(1,:)); mean(initBiasAccelList(2,:)); mean(initBiasAccelList(3,:)); ]

%% VIO pipeline
%Set parameters
xInit.p = zeros(3,1);
xInit.v = zeros(3,1);
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = imuData.measOrient(:,1);

%Pipeline
pipelineOptions.featureCount = 1000;
pipelineOptions.initDisparityThreshold = 5;
pipelineOptions.kfDisparityThreshold = 10;
pipelineOptions.showFeatureTracks = true;
pipelineOptions.inlierThreshold = 0.5^2;
pipelineOptions.inlierMinDisparity = 2;
pipelineOptions.inlierMaxForwardDistance = 20;
pipelineOptions.verbose = true;
pipelineOptions.g_norm = 9.8093;

g_w = [0 0 pipelineOptions.g_norm]';

%GTSAM
pipelineOptions.minViewingsForLandmark = 2;
pipelineOptions.obsNoiseSigma = 0.1;
pipelineOptions.useRobustMEst = true;
pipelineOptions.mEstWeight = 1;
pipelineOptions.triangPointSigma = 10;

noiseParams.sigma_g = 0.001*ones(3,1); 
noiseParams.sigma_a =  0.01*ones(3,1);
noiseParams.sigma_bg = [1e-5; 1e-5; 1e-5];
noiseParams.sigma_ba = [1e-5; 1e-5; 1e-5];
noiseParams.init_ba = [0.000169839585484; -0.000181405835863; -0.001097839079856];
noiseParams.init_bg = [0.0001; 0.0001; 0.0001];

%The pipeline
[T_wc_estimated,T_wimu_estimated, T_wimu_gtsam, keyFrames] = VIOPipelineV2_GTSAMCrucifix(K, T_camimu, monoImageData, imuData, pipelineOptions, noiseParams, xInit, g_w);

%

% Plot the estimated values
figure;
p_IMUw_w_GT = zeros(3, length(keyFrames));
%p_IMUw_w_int = zeros(3, length(keyFrames));
p_IMUw_w_gtsam = zeros(3, length(keyFrames));

% Get the keyframe IMU ids
keyFrameIdsIMU = zeros(1, length(keyFrames));
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