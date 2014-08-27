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
addpath('training');
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
%Roads, Rail, Trees
 dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0001_sync';
 dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%% Options
%Pipeline
pipelineOptions.featureDetector = 'SURF';
pipelineOptions.featureCount = 5000;
pipelineOptions.descriptorExtractor = 'SURF';
pipelineOptions.descriptorMatcher = 'BruteForce';
pipelineOptions.minMatchDistance = 0.1;


pipelineOptions.initDisparityThreshold = 1;
pipelineOptions.kfDisparityThreshold = 3;
pipelineOptions.showFeatureTracks = true;


pipelineOptions.inlierThreshold = 2^2;
pipelineOptions.inlierMinDisparity = 2;
pipelineOptions.inlierMaxForwardDistance = 50;

pipelineOptions.verbose = true;

% g2o options
g2oOptions.maxPixError = 100;
g2oOptions.fixLandmarks = false;
g2oOptions.fixPoses = false;
g2oOptions.motionEdgeInfoMat = inv(diag([0.01^2, 0.01^2,0.01^2, 0.000175^2, 0.000175^2, 0.000175^2]));
g2oOptions.obsEdgeInfoMat = 0.1*eye(2);



%% Get ground truth and import data
% frameNum limits the import to first frameNum frames (if this exceeds the
% number in the dataset, all frames are used)
frameRange = 1:1166;

%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir);

%Image data
%monoImageData = loadImageDataOpenCV([dataBaseDir '/image_00'], frameRange);

%IMU data
imuData = loadImuData(dataBaseDir, frameRange);

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



xInit.p = zeros(3,1);
xInit.v = imuData.initialVelocity;
xInit.b_g = zeros(3,1);
xInit.b_a = zeros(3,1);
xInit.q = [1;zeros(3,1)];

g_w = -1*rotmat_from_quat(imuData.measOrient(:,1))'*[0 0 9.81]';
noiseParams.sigma_g = 4e-4;
noiseParams.sigma_a = 2e-3;
noiseParams.sigma_bg = 3e-3;
noiseParams.sigma_ba = 8e-5;
noiseParams.tau = 3600;

 
%Initialize
currentPoseGlobal = Pose3(Rot3(), Point3()); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector(xInit.v); 
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
noiseModelGPS = noiseModel.Diagonal.Precisions([ [0;0;0]; 1.0/0.00007 * [1;1;1] ]);

IMU_metadata.AccelerometerBiasSigma = 0.000167;
IMU_metadata.GyroscopeBiasSigma = 2.91e-006;
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
w_coriolis = [0;0;0];

% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(1);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

%Extract intrinsics
% f_x = K(1,1);
% f_y = K(2,2);
% c_x = K(1,3);
% c_y = K(2,3);
% 
% 
% % Create realistic calibration and measurement noise model
% % format: fx fy skew cx cy baseline
% K_gtsam = Cal3_S2(f_x, f_y, 0, c_x, c_y);
% mono_model_n = noiseModel.Diagonal.Sigmas([0.1,0.1]');


%landmarks struct:
% landmarks.id %1xN
% landmarks.position %3xN

% Create initial estimate and prior on initial pose, velocity, and biases
currentPoseKey = symbol('x',1);
currentVelKey =  symbol('v',1);
currentBiasKey = symbol('b',1);
newValues.insert(currentPoseKey, currentPoseGlobal);
newValues.insert(currentVelKey, currentVelocityGlobal);
newValues.insert(currentBiasKey, currentBias);
newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));

currState = xInit;

T_wIMU_list_opt(:,:,1) = currentPoseGlobal.matrix;
T_wIMU_list_int(:,:,1) = currentPoseGlobal.matrix;

for keyId = 2:floor((length(imuData.timestamps))/10)

  % At each non=IMU measurement we initialize a new node in the graph
  currentPoseKey = symbol('x',keyId);
  currentVelKey =  symbol('v',keyId);
  currentBiasKey = symbol('b',keyId);
  
      % Summarize IMU data between the previous GPS measurement and now
    currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
      currentBias, 0.01.^2 * eye(3), ...
      0.000175.^2 * eye(3), 0 * eye(3));

      IMUindices = ((keyId-1)*10 + 1):(keyId*10);
  
      for imuMeasId = IMUindices
  
  %Integrate forward
  %imuMeasId:imuMeasId+5
  dt = imuData.timestamps(imuMeasId) - imuData.timestamps(imuMeasId-1);
  %Extract the measurements
  imuAccel = imuData.measAccel(:, imuMeasId);
  imuOmega = imuData.measOmega(:, imuMeasId);

   %Predict the next state
  [currState] = integrateIMU(currState, imuAccel, imuOmega, dt, noiseParams, g_w);    
      %Update measurements  
      accMeas = [imuAccel];
      omegaMeas = [imuOmega];
      deltaT = dt;
      currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);

      end
      
        currPose = Pose3(Rot3(rotmat_from_quat(currState.q)), Point3(currState.p));
      
      
    % Create IMU factor
    newFactors.add(ImuFactor( ...
      currentPoseKey-1, currentVelKey-1, ...
      currentPoseKey, currentVelKey, ...
      currentBiasKey, currentSummarizedMeasurement, g_w, w_coriolis));

    % Bias evolution as given in the IMU metadata
    newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
      noiseModel.Diagonal.Sigmas(1 * sigma_between_b)));
        
        if mod(keyId, 5) == 0
            newFactors.add(PriorFactorPose3(currentPoseKey, currPose, noiseModelGPS));
        end

    newValues.insert(currentPoseKey, currPose);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    

    if keyId > 10
    isam.update(newFactors, newValues);
    newFactors = NonlinearFactorGraph;
    newValues = Values;
    T_wIMU_list_opt(:,:,keyId) = isam.calculateEstimate(currentPoseKey).matrix;
    currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
    currentBias = isam.calculateEstimate(currentBiasKey);
    else
    T_wIMU_list_opt(:,:,keyId) = currPose.matrix;
    end
    prevPose = currPose;

    T_wIMU_list_int(:,:,keyId) = currPose.matrix;


end




% Plot the result
for i = 1:size(T_wIMU_list_opt, 3)
    T_wc_list_opt(:,:,i) = T_wIMU_list_opt(:,:,i)*inv(T_camimu);
    T_wc_list_int(:,:,i) = T_wIMU_list_int(:,:,i)*inv(T_camimu);
end


figure;
p_CAMw_w_GT = zeros(3, size(T_wCam_GT,3));
p_CAMw_w_int = zeros(3, size(T_wc_list_opt,3));
p_CAMw_w_estOpt = zeros(3, size(T_wc_list_opt,3));

for i = 1:size(p_CAMw_w_GT,2)
    p_CAMw_w_GT(:,i) = homo2cart(T_wCam_GT(:,:,i)*[0;0;0;1]);
end
for i = 1:size(p_CAMw_w_estOpt, 2)
    p_CAMw_w_estOpt(:,i) = homo2cart(T_wc_list_opt(:,:,i)*[0;0;0;1]);
    p_CAMw_w_int(:,i) = homo2cart(T_wc_list_int(:,:,i)*[0;0;0;1]);
end


plot3(p_CAMw_w_GT(1,:),p_CAMw_w_GT(2,:),p_CAMw_w_GT(3,:), '.-k');
hold on; grid on;
plot3(p_CAMw_w_estOpt(1,:),p_CAMw_w_estOpt(2,:),p_CAMw_w_estOpt(3,:), '.-g');
plot3(p_CAMw_w_int(1,:),p_CAMw_w_int(2,:),p_CAMw_w_int(3,:), '.-r');


xlabel('X'); ylabel('Y'); zlabel('Z');
%daspect([]);
view([0 90]);

legend('Ground Truth', 'IMU Only','VIO', 4);

%%
% Calculate Relative Pose Error
% Take only the poses at the keyframes
T_wCam_GT_sync = T_wCam_GT(:,:,keyFrameIds);
T_wc_est_sync = T_wc_estimated(:,:, keyFrameIds);

dstep = 1;

RPE_opt =  zeros(4,4, size(T_wCam_GT_sync,3) - dstep);
RPE_imuonly = RPE_opt;

for i = 1:(size(T_wCam_GT_sync,3)-dstep)
    RPE_opt(:,:,i) = inv(inv(T_wCam_GT_sync(:,:,i))*T_wCam_GT_sync(:,:,i+dstep))*inv(T_wc_list_opt(:,:,i))*T_wc_list_opt(:,:,i+dstep); 
    RPE_imuonly(:,:,i) = inv(inv(T_wCam_GT_sync(:,:,i))*T_wCam_GT_sync(:,:,i+dstep))*inv(T_wc_est_sync(:,:,i))*T_wc_est_sync(:,:,i+dstep);  
end

%Calculate the root mean squared error of all the relative pose errors
RMSE_RPE_opt = 0;
RMSE_RPE_imuonly = 0;

for i = 1:size(RPE_opt,3)
    RMSE_RPE_opt = RMSE_RPE_opt + norm(RPE_opt(1:3,4,i))^2;
    RMSE_RPE_imuonly = RMSE_RPE_imuonly + norm(RPE_imuonly(1:3,4,i))^2;  
end
RMSE_RPE_imuonly = sqrt(RMSE_RPE_imuonly/size(RPE_opt,3));
RMSE_RPE_opt = sqrt(RMSE_RPE_opt/size(RPE_opt,3));

%Add to the title
title(sprintf('RMSE RPE (Optimized/IMU Only): %.5f / %.5f ', RMSE_RPE_opt, RMSE_RPE_imuonly));


printf('--------- \n End Euclidian Error (Opt/IMU): %.5f / %.5f', norm(p_CAMw_w_GT(:,end) -  p_CAMw_w_estOpt(:, end)) ,norm(p_CAMw_w_GT(:,end) -  p_CAMw_w_est(:, end)));

% Display mean errors
opt_errors = p_CAMw_w_GT -  p_CAMw_w_estOpt;
imu_errors = p_CAMw_w_GT -  p_CAMw_w_est;

mean_opt_euc = mean(vecNorms(opt_errors));
mean_imu_euc = mean(vecNorms(imu_errors));

printf('--------- \n Mean Euclidian Error (Opt/IMU): %.5f / %.5f',mean_opt_euc , mean_imu_euc);

