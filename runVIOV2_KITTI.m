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
if ismac
    addpath('/Users/valentinp/Research/opengv/matlab');
    addpath('/Users/valentinp/Research/mexopencv');
else
    addpath('~/Dropbox/Research/Ubuntu/opengv/matlab');
    addpath('~/mexopencv/');
end

if ismac
    addpath('/Users/valentinp/Research/gtsam_toolbox');
else
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
end
 import gtsam.*;

%% Where is the data?
%Karslrugh city centre
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_29/2011_09_29_drive_0071_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_29';

%Open street
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0036_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';
 
 %Foresty road
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0028_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

 %Cityish
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0059_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%Trail through forest
 %dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0087_sync';
 %dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%Turn
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_30/2011_09_30_drive_0027_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_30';

%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0002_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_30/2011_09_30_drive_0027_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_30';

%dataBaseDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_26/2011_09_26_drive_0002_sync';
%dataCalibDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_26';

%Train
dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0036_sync';
dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0023_extract';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%dataBaseDir = '/home/valentin/Desktop/KITTI/2011_10_03/2011_10_03_drive_0042_extract';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_10_03';

%dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0035_extract';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';


%% Get ground truth and import data
frameRange = 1:200;

%Image data
monoImageData = loadImageData([dataBaseDir '/image_00'], frameRange);

%IMU data
[imuData, imuFrames] = loadImuData(dataBaseDir, monoImageData.timestamps);
%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir, imuFrames);

%Decimate monoImageData
monoImageData.timestamps(2:2:end) = [];
monoImageData.rectImages(:,:,2:2:end) = [];

%% Load calibration
[T_camvelo_struct, K] = loadCalibration(dataCalibDir);
T_camvelo = T_camvelo_struct{1}; %We are using camera 1 (left rect grayscale)
T_veloimu = loadCalibrationRigid(fullfile(dataCalibDir,'calib_imu_to_velo.txt'));
T_camimu = T_camvelo*T_veloimu;

%Add camera ground truth

T_wCam_GT = T_wIMU_GT;

for i = 1:size(T_wIMU_GT, 3)
    T_wCam_GT(:,:,i) = T_wIMU_GT(:,:,i)*inv(T_camimu);
end


%% VIO pipeline
%Set parameters
close all;
%Pipeline
pipelineOptions.featureCount = 1000;
pipelineOptions.initDisparityThreshold = 1;
pipelineOptions.kfDisparityThreshold = 10;
pipelineOptions.showFeatureTracks = false;


pipelineOptions.inlierThreshold = 0.5^2;
pipelineOptions.inlierMinDisparity = 3;
pipelineOptions.inlierMaxForwardDistance = 200;
pipelineOptions.verbose = true;

%GTSAM
pipelineOptions.minViewingsForLandmark = 3;
pipelineOptions.obsNoiseSigma = 0.5;
pipelineOptions.useRobustMEst = true;

pipelineOptions.mEstWeight = 5;
pipelineOptions.triangPointSigma = 10;



xInit.p = T_wIMU_GT(1:3,4,1);
xInit.v = imuData.initialVelocity;
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1;zeros(3,1)];

g_w = -1*rotmat_from_quat(imuData.measOrient(:,1))'*[0 0 9.81]';

noiseParams.sigma_g = 1e-3*ones(3,1); 
noiseParams.sigma_a =  1e-2*ones(3,1);
noiseParams.sigma_bg = 1e-5;
noiseParams.sigma_ba = 1e-5;
noiseParams.init_ba = zeros(3,1);
noiseParams.init_bg = zeros(3,1);
noiseParams.tau = 10^12;

 
%The pipeline
[T_wc_estimated,T_wimu_estimated, T_wimu_gtsam, keyFrames] = VIOPipelineV2_GTSAM(K, T_camimu, monoImageData, imuData, pipelineOptions, noiseParams, xInit, g_w);

%%
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

RMSE_RPE_opt = calcRelativePoseError( T_wIMU_GT_sync, T_wimu_gtsam );
RMSE_RPE_imuonly = calcRelativePoseError( T_wIMU_GT_sync, T_wimu_est_sync );


%Add to the title
title(sprintf('Mean RMSE RPE (Optimized/IMU Only): %.5f / %.5f ', RMSE_RPE_opt, RMSE_RPE_imuonly));

printf('--------- \n End Euclidian Error (Opt/IMU): %.5f / %.5f', norm(p_IMUw_w_GT(:,end) -  p_IMUw_w_gtsam(:, end)) ,norm(p_IMUw_w_GT(:,end) -  p_IMUw_w_int(:, end)));

% Display mean errors
opt_errors = p_IMUw_w_GT -  p_IMUw_w_gtsam;
imu_errors = p_IMUw_w_GT -  p_IMUw_w_int;

mean_opt_euc = mean(sqrt(sum(opt_errors.^2, 1)));
mean_imu_euc = mean(sqrt(sum(imu_errors.^2, 1)));

printf('--------- \n Mean Euclidian Error (Opt/IMU): %.5f / %.5f',mean_opt_euc , mean_imu_euc);
