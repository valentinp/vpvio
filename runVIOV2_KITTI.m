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
% dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0087_sync';
% dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

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
%dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0001_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0023_extract';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

dataBaseDir = '/home/valentin/Desktop/KITTI/2011_10_03/2011_10_03_drive_0042_extract';
dataCalibDir = '/home/valentin/Desktop/KITTI/2011_10_03';


%% Options
%Pipeline
pipelineOptions.featureDetector = 'SURF';
pipelineOptions.featureCount = 1000;
pipelineOptions.descriptorExtractor = 'SURF';
pipelineOptions.descriptorMatcher = 'FlannBased';
pipelineOptions.minMatchDistance = 0.4;


pipelineOptions.initDisparityThreshold = 1;
pipelineOptions.kfDisparityThreshold = 5;
pipelineOptions.showFeatureTracks = true;


pipelineOptions.inlierThreshold = 1^2;
pipelineOptions.inlierMinDisparity = 1;
pipelineOptions.inlierMaxForwardDistance = 100;

pipelineOptions.verbose = true;

% g2o options
g2oOptions.maxPixError = 25;
g2oOptions.fixLandmarks = false;
g2oOptions.fixPoses = false;
g2oOptions.motionEdgeInfoMat = inv(diag([0.01^2, 0.01^2,0.01^2, 0.000175^2, 0.000175^2, 0.000175^2]));
g2oOptions.obsEdgeInfoMat = 4*eye(2);



%% Get ground truth and import data
% frameNum limits the import to first frameNum frames (if this exceeds the
% number in the dataset, all frames are used)
frameRange = 1:75;

%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir);
%frameNum = min(size(T_wIMU_GT,3), frameNum);
%T_wIMU_GT = T_wIMU_GT(:,:,frameRange);

%Image data
monoImageData = loadImageData([dataBaseDir '/image_00'], frameRange);

%IMU data
imuData = loadImuData(dataBaseDir, 1:1000);

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

xInit.p = T_wIMU_GT(1:3,4,1);
xInit.v = imuData.initialVelocity;
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1;zeros(3,1)];

g_w = -1*rotmat_from_quat(imuData.measOrient(:,1))'*[0 0 9.81]';
noiseParams.sigma_g = 0;
noiseParams.sigma_a = 0;
noiseParams.sigma_bg = 0;
noiseParams.sigma_ba = 0;
noiseParams.tau = 10^12;

 
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

% %% G2O
% % Extract unique landmarks
% landmarks.id = [];
% landmarks.position = [];
% landmarks.count = [];
% 
% keyFrameIdsIMU = zeros(1, length(keyFrames));
% keyFrameIdsCAM = zeros(1, length(keyFrames));
% 
% totalLandmarks = 0;
% 
% for i = 1:length(keyFrames)
%      kf = keyFrames(i);
%      
%     landmarkIds = kf.landmarkIds;
%     landmarkPositions = kf.pointCloud;
%     totalLandmarks = totalLandmarks + size(landmarkPositions, 2);
%     
%     %Deprecated version
% %     [newLandmarkIds,idx] = setdiff(landmarkIds,landmarks.id);
% %     landmarks.id = [landmarks.id newLandmarkIds];
% %     landmarks.position = [landmarks.position landmarkPositions(:, idx)];
% 
%     %New version: landmarks initialized to mean position
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
%     
%     keyFrameIdsIMU(i) = kf.imuMeasId;
%     keyFrameIdsCAM(i) = kf.camMeasId;
%     
% end
% 
% landmarks.position = landmarks.position./repmat(landmarks.count, [3,1]);
% 
% disp(['Total Keyframes: ' num2str(length(keyFrames))]);
% disp(['Total unique tracked landmarks: ' num2str(length(landmarks.id))]);
% disp(['Total triangulated landmarks: ' num2str(totalLandmarks)]);
% 
% %
% close all
% visualizeVO([], T_wc_estimated(:,:,keyFrameIdsIMU), landmarks.position, '- Non Optimized')
% 
% %%
% %Use GTSAM?
% %[T_wc_list_opt, landmarks_w_opt] = processWithGTSAM(keyFrames,landmarks, K, g2oOptions);
% 
% 
% % Optimize the result
% if exist('keyframes.g2o', 'file') == 2
% delete('keyframes.g2o');
% end
% if exist('opt_keyframes.g2o', 'file') == 2
% delete('opt_keyframes.g2o');
% end
% 
% close all;
% visualizeVO([], T_wc_estimated(:,:,keyFrameIdsIMU), landmarks.position, '- Non Optimized')
% 
% exportG2ODataExpMap(keyFrames,landmarks, K, 'keyframes.g2o', g2oOptions)
% 
% %command = '!g2o_bin/g2o -i 1000  -  v -robustKernel DCS -solver   lm_dense6_3 -o opt_keyframes.g2o test.g2o';
% %command = '!g2o_bin/g2o -i 25 -v -solver lm_var -solverProperties initialLambda=0.001 -o -printSolverProperties opt_keyframes.g2o test.g2o';
% %-robustKernel Cauchy -robustKernelWidth 1
% 
% %Check if we're using Ubuntu or OSX
% if ismac
%     g2o_exec = '!g2o_bin/g2o_mac';
% else
%     g2o_exec = '!g2o_bin/g2o';
% end
% command = sprintf('%s -i 100 -v -solver  lm_var  -o  opt_keyframes.g2o keyframes.g2o', g2o_exec);
% eval(command);
% 
% 
% %command = '!g2o_bin/g2o -i 100 -v -solver lm_dense6_3  -o  opt_keyframes.g2o keyframes.g2o';
% 
% [T_wc_list_opt, landmarks_w_opt] = importG2ODataExpMap('opt_keyframes.g2o');
% 
% visualizeVO([], T_wc_list_opt, landmarks_w_opt, '- Optimized')
% 
% % Plot the result
% figure;
% p_CAMw_w_GT = zeros(3, size(T_wCam_GT,3));
% p_CAMw_w_estOpt = zeros(3, size(T_wc_list_opt,3));
% p_CAMw_w_est = zeros(3, size(T_wc_estimated,3));
% 
% for i = 1:size(p_CAMw_w_GT,2)
%     p_CAMw_w_GT(:,i) = homo2cart(T_wCam_GT(:,:,i)*[0;0;0;1]);
%     p_CAMw_w_est(:,i) = homo2cart(T_wc_estimated(:,:,i)*[0;0;0;1]);
% end
% for i = 1:size(p_CAMw_w_estOpt, 2)
%     p_CAMw_w_estOpt(:,i) = homo2cart(T_wc_list_opt(:,:,i)*[0;0;0;1]);
% end
% 
% p_CAMw_w_GT = p_CAMw_w_GT(:, keyFrameIdsIMU);
% p_CAMw_w_est = p_CAMw_w_est(:, keyFrameIdsIMU);
% 
% 
% plot3(p_CAMw_w_GT(1,:),p_CAMw_w_GT(2,:),p_CAMw_w_GT(3,:), '.-k');
% hold on; grid on;
% plot3(p_CAMw_w_est(1,:), p_CAMw_w_est(2,:), p_CAMw_w_est(3,:),'.-r');
% plot3(p_CAMw_w_estOpt(1,:),p_CAMw_w_estOpt(2,:),p_CAMw_w_estOpt(3,:), '.-g');
% 
% view([0 90]);
% 
% legend('Ground Truth', 'IMU Only','VIO', 4);
% 
% % Calculate Relative Pose Error
% % Take only the poses at the keyframes
% T_wCam_GT_sync = T_wCam_GT(:,:,keyFrameIdsIMU);
% T_wc_est_sync = T_wc_estimated(:,:, keyFrameIdsIMU);
% 
% RMSE_RPE_imuonly_list = zeros(1, size(T_wCam_GT_sync,3)-1);
% RMSE_RPE_opt_list =zeros(1, size(T_wCam_GT_sync,3)-1);
% 
% %Iterative through different step sizes
% for dstep = 1:size(T_wCam_GT_sync,3)-1
% 
% RPE_opt =  zeros(4,4, size(T_wCam_GT_sync,3) - dstep);
% RPE_imuonly = RPE_opt;
% 
% for i = 1:(size(T_wCam_GT_sync,3)-dstep)
%     RPE_opt(:,:,i) = inv(inv(T_wCam_GT_sync(:,:,i))*T_wCam_GT_sync(:,:,i+dstep))*inv(T_wc_list_opt(:,:,i))*T_wc_list_opt(:,:,i+dstep); 
%     RPE_imuonly(:,:,i) = inv(inv(T_wCam_GT_sync(:,:,i))*T_wCam_GT_sync(:,:,i+dstep))*inv(T_wc_est_sync(:,:,i))*T_wc_est_sync(:,:,i+dstep);  
% end
% 
% %Calculate the root mean squared error of all the relative pose errors
% RMSE_RPE_opt = 0;
% RMSE_RPE_imuonly = 0;
% 
% for i = 1:size(RPE_opt,3)
%     RMSE_RPE_opt = RMSE_RPE_opt + norm(RPE_opt(1:3,4,i))^2;
%     RMSE_RPE_imuonly = RMSE_RPE_imuonly + norm(RPE_imuonly(1:3,4,i))^2;  
% end
% RMSE_RPE_imuonly_list(dstep) = sqrt(RMSE_RPE_imuonly/size(RPE_opt,3));
% RMSE_RPE_opt_list(dstep) = sqrt(RMSE_RPE_opt/size(RPE_opt,3));
% end
% 
% RMSE_RPE_opt = mean(RMSE_RPE_opt_list);
% RMSE_RPE_imuonly = mean(RMSE_RPE_imuonly_list);
% 
% %Add to the title
% title(sprintf('Mean RMSE RPE (Optimized/IMU Only): %.5f / %.5f ', RMSE_RPE_opt, RMSE_RPE_imuonly));
% 
% 
% printf('--------- \n End Euclidian Error (Opt/IMU): %.5f / %.5f', norm(p_CAMw_w_GT(:,end) -  p_CAMw_w_estOpt(:, end)) ,norm(p_CAMw_w_GT(:,end) -  p_CAMw_w_est(:, end)));
% 
% % Display mean errors
% opt_errors = p_CAMw_w_GT -  p_CAMw_w_estOpt;
% imu_errors = p_CAMw_w_GT -  p_CAMw_w_est;
% 
% mean_opt_euc = mean(sqrt(sum(opt_errors.^2, 1)));
% mean_imu_euc = mean(sqrt(sum(imu_errors.^2, 1)));
% 
% printf('--------- \n Mean Euclidian Error (Opt/IMU): %.5f / %.5f',mean_opt_euc , mean_imu_euc);

