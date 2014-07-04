clc;
clear all;
close all;
%Add paths
addpath('/Users/valentinp/Dropbox/UTIAS - MASc/Code/MATLAB/utils');
addpath('/Users/valentinp/Dropbox/UTIAS - MASc/Code/MATLAB/kinematics_toolbox/screws');
addpath('/Users/valentinp/Dropbox/UTIAS - MASc/Code/opengv/matlab');
addpath('helpers/');
addpath('kitti/');
addpath('kitti/devkit/');
%Set Data Path
dataBaseDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_29/2011_09_29_drive_0071_sync';
%dataBaseDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_26/2011_09_26_drive_0002_sync';
%dataBaseDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_26/2011_09_26_drive_0002_sync';
%dataBaseDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_26/2011_09_26_drive_0002_sync';


%% Get ground truth and import data
% frameNum limits the import to first frameNum frames (if this exceeds the
% number in the dataset, all frames are used)
frameNum = 300;

T_wIMU_GT = getGroundTruth(dataBaseDir);
frameNum = min(size(T_wIMU_GT,3), frameNum);
T_wIMU_GT = T_wIMU_GT(:,:,1:frameNum);

[monoImageData, dateNums] = loadImageData([dataBaseDir '/image_00'], frameNum);

% Calculate distance travelled to determine visual scale
dtList = diff(dateNums)*3600*24; %dateNums are in units of days - need to convert to seconds
dtList = [0; dtList];
distTravelled = integrateVelocity(dataBaseDir, dtList);

% Calculate ground truth distance travelled for reference
distTravelled_gt = zeros(1, size(T_wIMU, 3));
for i = 2:size(T_wIMU, 3)
    distTravelled_gt(i) = norm(T_wIMU(1:3,4,i));
end

%% Load calibration
[T_camvelo_struct, K] = loadCalibration(dataBaseDir);
T_camvelo = T_camvelo_struct{1}; %We are using camera 1 (left rect grayscale)
T_veloimu = loadCalibrationRigid(fullfile(dataBaseDir,'calib_imu_to_velo.txt'));
T_camimu = T_camvelo*T_veloimu;

%% Extract rotation priors
R_camimu = T_camimu(1:3,1:3);
R_wc_imu = zeros(3,3, length(monoImageData));

for i = 1:length(monoImageData)
    %T_worldIMU contains R_worldimu (where the world frame is the first frame in the sequence)
    %Thus R_wc = R_worldimu*(R_camimu)';
    R_wc_imu(:,:,i) = T_wIMU(1:3,1:3,i)*(R_camimu');
end

%% Run Pipeline

%Set parameters
params = {};
params.INIT_DISPARITY_THRESHOLD = 10^2; %in squared pixels
params.KF_DISPARITY_THRESHOLD = 2^2; %in squared pixels
params.MIN_FEATURE_MATCHES = 3;
params.LIVE_PLOT = false;
params.VERBOSE = false;

%Launch Pipeline
tic();
T_wc_estimated = VIOPipeline(K, monoImageData, R_wc_imu, params);
toc
%Transform back into the imu frame
T_wIMU_est = T_wc_estimated;
 for i = 1:size(T_wIMU_est,3)
     T_wIMU_est(1:3,1:3,i) = T_wc_estimated(1:3,1:3,i)*R_camimu;
 end

%Downsample
%Take every 1th transform
T_wIMU_ds = downSampleTransform(T_wIMU, 1);
T_wimu_est_ds = downSampleTransform(T_wIMU_est, 1);

 
%% Visualize
% Get the 2D paths
scale = 5.5;
p_wIMU_est = T_wimu_est_ds(1:2, 4, :);
p_wIMU = T_wIMU_ds(1:2, 4, :);

%Reshape in 2xN matrices
p_wIMU_est = scale*reshape(p_wIMU_est, [2, size(p_wIMU_est,3)]);
p_wIMU = reshape(p_wIMU, [2, size(p_wIMU, 3)] );

%Plot 2D VIO and Ground truth
close all;
figure;
plot(p_wIMU_est(1,:), p_wIMU_est(2,:), '-r', 'LineWidth', 2);
hold on;
plot(p_wIMU(1,:), p_wIMU(2,:), '-b', 'LineWidth', 2);
xlabel('x (m)', 'FontSize', 16);
ylabel('y (m)', 'FontSize', 16);
legend('VIO', 'Ground Truth', 2);
set(gca,'FontSize',16);
titleList =  strsplit(dataBaseDir, '/'); %Use the folder name for the data as a title
plotTitle = titleList(end);
title(plotTitle,'interpreter','none', 'FontSize', 16);


% OLD METHOD: DRAW FRAME TRAJECTORIES
% close all;
% figure;
% drawframetraj(T_wIMU_ds)
% hold on;
% drawframetraj(T_wimu_est_ds)
