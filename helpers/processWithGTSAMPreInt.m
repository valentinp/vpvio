function [T_wc_list_opt, landmarks_w_opt] = processWithGTSAMPreInt(keyFrames, landmarks, K, g2oOptions)

%Reset the count variable
landmarks.count = zeros(length(landmarks.id), 1);
insertedLandmarkIds = [];

if ismac
    addpath('/Users/valentinp/Research/gtsam_toolbox');
else
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
end
 import gtsam.*;
%Add observations to the previous keyframe
for i = 2:length(keyFrames)
    prevKf = keyFrames(i-1);
    kf = keyFrames(i);
    [newLandmarkIds,idx] = setdiff(kf.landmarkIds, prevKf.landmarkIds);
     keyFrames(i-1).landmarkIds = [keyFrames(i-1).landmarkIds newLandmarkIds];
     keyFrames(i-1).pixelMeasurements = [keyFrames(i-1).pixelMeasurements kf.refPosePixels(:, idx)];
     keyFrames(i-1).predVectors = [keyFrames(i-1).predVectors kf.predVectors(:, idx)];
     
end


%Remove single observation landmarks (these will be all in the first
%keyframe)
allLandmarkIds = [];
for i = 1:length(keyFrames)
    kf = keyFrames(i);
    allLandmarkIds = [allLandmarkIds kf.landmarkIds];
end
[uniques,numUnique] = count_unique(allLandmarkIds);
singleObsLandmarkIds =  uniques(numUnique < 2);


%Remove all single observations in the first keyframe
for l_id = 1:length(keyFrames(1).landmarkIds)
   landmarkId = keyFrames(1).landmarkIds(l_id);
   if ismember(landmarkId, singleObsLandmarkIds)
        keyFrames(1).landmarkIds(l_id) = NaN;
        keyFrames(1).pixelMeasurements(:,l_id) = [NaN NaN]';
        keyFrames(1).predVectors(:,l_id) = NaN*ones(size(keyFrames(1).predVectors, 1),1);
        
        landmarks.position(:, landmarks.id == landmarkId) = [NaN NaN NaN]';
        landmarks.id(landmarks.id == landmarkId) = NaN;
   end
end

 keyFrames(1).pixelMeasurements(:, isnan(keyFrames(1).pixelMeasurements(1,:))) = [];
  keyFrames(1).predVectors(:, isnan(keyFrames(1).predVectors(1,:))) = [];

  keyFrames(1).landmarkIds(isnan(keyFrames(1).landmarkIds)) = [];
  
  
 landmarks.position(:, isnan(landmarks.position(1,:))) = [];
  landmarks.id(isnan(landmarks.id)) = [];


%Eliminate all crazy pixel errors
deleteObservations = []; 
landmarkIds_all = [];
pix_error_all = [];
for keyFrameNum = 1:length(keyFrames)
deleteObservations(keyFrameNum).deleteObs = [];

for l_id = 1:length(keyFrames(keyFrameNum).landmarkIds)
    landmarkId = keyFrames(keyFrameNum).landmarkIds(l_id);
    T_wk = keyFrames(keyFrameNum).T_wk;
    landmark_pos_w = landmarks.position(:,landmarks.id == landmarkId);
    landmark_pos_k = homo2cart(inv(T_wk)*[landmark_pos_w; 1]);
    pixel_coords = homo2cart(K*landmark_pos_k);
    true_pixel_coords = keyFrames(keyFrameNum).pixelMeasurements(:, l_id);
    pix_error = norm(pixel_coords - true_pixel_coords);
    landmarkIds_all = [landmarkIds_all landmarkId];
    
    if pix_error > g2oOptions.maxPixError
            deleteObservations(keyFrameNum).deleteObs(end+1) = l_id;
    else
          pix_error_all = [pix_error_all pix_error];
    end
end
end

totalDeletions = 0;
for kf = 1:length(deleteObservations)
    %Delete bad observations in this keyframe
    if ~isempty(deleteObservations(kf).deleteObs) 
        l_ids = deleteObservations(kf).deleteObs;
        keyFrames(kf).pixelMeasurements(:, l_ids) = [];
        keyFrames(kf).predVectors(:, l_ids) = [];
        keyFrames(kf).landmarkIds(l_ids) = [];
        totalDeletions = totalDeletions + length(l_ids);
    end
end
printf(['--------- \nDeleted ' num2str(totalDeletions) ' bad observations.\n---------\n']);


%Final cleanup: ensure there are no landmarks that have less than 2
%observations and remove any landmarks that are not in our good clusters

allLandmarkIds = [];
for i = 1:length(keyFrames)
    kf = keyFrames(i);
    allLandmarkIds = [allLandmarkIds kf.landmarkIds];
end
[uniques,numUnique] = count_unique(allLandmarkIds);
singleObsLandmarkIds =  uniques(numUnique < 4);
noObsLandmarkIds = setdiff(landmarks.id, uniques);
badLandmarkIds = union(singleObsLandmarkIds, noObsLandmarkIds);

badLandmarkIds = [badLandmarkIds; 100072; 100021; 100096];

%Output variables
T_wc_list_opt = zeros(4,4,length(keyFrames));
landmarks_w_opt = zeros(3, length(landmarks.id));


%Initialize
kf = keyFrames(1);
R_wk = kf.R_wk;
t_kw_w = kf.t_kw_w;
currentPoseGlobal = Pose3(Rot3(R_wk), Point3(t_kw_w)); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector(keyFrames(1).imuDataStruct{1}.v); 
%currentPoseGlobal = Pose3(Rot3, Point3); % initial pose is the reference frame (navigation frame)
%currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
noiseModelGPS = noiseModel.Diagonal.Precisions([ [0;0;0]; 1.0/0.07 * [1;1;1] ]);

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
f_x = K(1,1);
f_y = K(2,2);
c_x = K(1,3);
c_y = K(2,3);

% Create realistic calibration and measurement noise model
% format: fx fy skew cx cy baseline
K = Cal3_S2(f_x, f_y, 0, c_x, c_y);
mono_model_n = noiseModel.Diagonal.Sigmas([0.1,0.1]');


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

T_wc_list_opt(:,:,1) = currentPoseGlobal.matrix;

landmarkIds = keyFrames(1).landmarkIds;
pixelMeasurements = keyFrames(1).pixelMeasurements;
for j = 1:length(landmarkIds)
        if ~ismember(landmarkIds(j), badLandmarkIds) 
        if ~newValues.exists(landmarkIds(j)) 
        newValues.insert(landmarkIds(j), Point3(double(landmarks.position(:,landmarks.id == landmarkIds(j)))));
                insertedLandmarkIds = [insertedLandmarkIds landmarkIds(j)];
        end
        
        %Update the usage count
        landmarks.count(landmarks.id == landmarkIds(j)) = landmarks.count(landmarks.id == landmarkIds(j)) + 1;
        
        newFactors.add(GenericProjectionFactorCal3_S2(Point2(double(pixelMeasurements(:,j))), mono_model_n, symbol('x',1), landmarkIds(j), K));
       end
end

    
for i = 2:length(keyFrames)
    kf = keyFrames(i);
    R_wk = kf.R_wk;
    t_kw_w = kf.t_kw_w;
    
    pixelMeasurements = kf.pixelMeasurements;
    landmarkIds = kf.landmarkIds;
    
      % At each non=IMU measurement we initialize a new node in the graph
  currentPoseKey = symbol('x',i);
  currentVelKey =  symbol('v',i);
  currentBiasKey = symbol('b',i);

           
    
    currPose = Pose3(Rot3(R_wk), Point3(t_kw_w));
    if i == 2
        T_wc_list_opt(:,:,2) = currPose.matrix;
    end
    %Fix the first pose 
    % Summarize IMU data between the previous GPS measurement and now
    currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
      currentBias, 0.01.^2 * eye(3), ...
      0.000175.^2 * eye(3), 0 * eye(3));

      %Update measurements 
      for imuMeasId = 1:length(kf.imuDataStruct)
      accMeas = [kf.imuDataStruct{imuMeasId}.a ];
      omegaMeas = [kf.imuDataStruct{imuMeasId}.omega ];
      deltaT = kf.imuDataStruct{imuMeasId}.dt;
      currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
      end
      
            
          
    % Create IMU factor
    newFactors.add(ImuFactor( ...
      currentPoseKey-1, currentVelKey-1, ...
      currentPoseKey, currentVelKey, ...
      currentBiasKey, currentSummarizedMeasurement, kf.imuDataStruct{1}.g_w, w_coriolis));

    % Bias evolution as given in the IMU metadata
    newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
      noiseModel.Diagonal.Sigmas(1 * sigma_between_b)));
  
      %newFactors.add(PriorFactorPose3(currentPoseKey, currPose, noiseModelGPS));

  
%            delta = prevPose.between(currPose);
%          covariance = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01; 0.000175; 0.000175; 0.000175]);
%          newFactors.add(BetweenFactorPose3(currentPoseKey-1,currentPoseKey, delta, covariance));        

    newValues.insert(currentPoseKey, currPose);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);

    for j = 1:length(landmarkIds)
     if ~ismember(landmarkIds(j), badLandmarkIds)   
        if ismember(i, [4 6 8]) && landmarks.count(landmarks.id == landmarkIds(j)) > 0 || ismember(i, [2 3 5 7 9])
        if ~ismember(landmarkIds(j), insertedLandmarkIds) 
        newValues.insert(landmarkIds(j), Point3(double(landmarks.position(:,landmarks.id == landmarkIds(j)))));
        insertedLandmarkIds = [insertedLandmarkIds landmarkIds(j)];
        end
        %Update the usage count
        landmarks.count(landmarks.id == landmarkIds(j)) = landmarks.count(landmarks.id == landmarkIds(j)) + 1;
        newFactors.add(GenericProjectionFactorCal3_S2(Point2(double(pixelMeasurements(:,j))), mono_model_n, symbol('x',i), landmarkIds(j), K));

        end
     end
    end

    if ismember(i, [4 6 8])
    
    landmarks.count(landmarks.count>0)
    landmarks.id(landmarks.count>0)
    
    isam.update(newFactors, newValues);
    newFactors = NonlinearFactorGraph;
    newValues = Values;
    T_wc_list_opt(:,:,i) = isam.calculateEstimate(currentPoseKey).matrix
    currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
    currentBias = isam.calculateEstimate(currentBiasKey);
    end


end


%newFactors = NonlinearFactorGraph;
%newValues = Values;

% for i = 1:length(landmarks.id)
%         if ~ismember(landmarks.id(i), badLandmarkIds)
%             newValues.insert(landmarks.id(i), Point3(double(landmarks.position(:,i))));
%         end
% end

   
end

