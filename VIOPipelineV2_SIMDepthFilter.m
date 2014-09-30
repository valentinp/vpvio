function [T_wcam_estimated,T_wimu_estimated,T_wimu_gtsam, keyFrames] = VIOPipelineV2_SIMDepthFilter(K, T_camimu, imageMeasurements, imuData, pipelineOptions, noiseParams, xInit, g_w, T_wImu_GT, landmarks_w)
%VIOPIPELINE Run the Visual Inertial Odometry Pipeline
% K: camera intrinsics
% T_camimu: transformation from the imu to the camera frame
% imuData: struct with IMU data:
%           imuData.timestamps: 1xN 
%           imuData.measAccel: 3xN
%           imuData.measOmega: 3xN
%           imuData.measOrient: 4xN (quaternion q_sw, with scalar in the
%           1st position. The world frame is defined as the N-E-Down ref.
%           frame.
% imageMeasurements:
%           array of imageMeasurement structs:
%           imageMeasurements(i).timestamp
%           imageMeasurements(i).pixelMeasurements (2xN)
%           imageMeasurements(i).landmarkIds (Nx1)

% params:
%           params.INIT_DISPARITY_THRESHOLD
%           params.KF_DISPARITY_THRESHOLD
%           params.MIN_FEATURE_MATCHES

 import gtsam.*;

%===GTSAM INITIALIATION====%
currentPoseGlobal = Pose3(Rot3(rotmat_from_quat(xInit.q)), Point3(xInit.p)); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector(xInit.v); 
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_v = noiseModel.Isotropic.Sigma(3, 0.1);
sigma_init_b = noiseModel.Isotropic.Sigmas([noiseParams.sigma_ba; noiseParams.sigma_bg]);
sigma_between_b = [ noiseParams.sigma_ba ; noiseParams.sigma_bg ];
w_coriolis = [0;0;0];



% Solver object
isamParams = ISAM2Params;
isamParams.setRelinearizeSkip(1);
gnParams = ISAM2GaussNewtonParams;
%gnParams.setWildfireThreshold(1000);
isamParams.setOptimizationParams(gnParams);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;
%==========================%

invK = inv(K);
% Main loop
% Keep track of key frames and poses
referencePose = {};

%Key frame poses correspond to the first and second poses from which 
%point clouds are triangulated (these must have sufficient disparity)
keyFrames = [];
keyFrame_i = 1;
initiliazationComplete = false;




% Main loop
% ==========================================================
% Sort all measurements by their timestamps, process measurements as if in
% real-time

%Extract image timestamps
imageTimestamps = zeros(1, length(imageMeasurements));
for i = 1:length(imageMeasurements)
    imageTimestamps(i) = imageMeasurements(i).timestamp;
end

%All measurements are assigned a unique measurement ID based on their
%timestamp
numImageMeasurements = length(imageTimestamps);
numImuMeasurements = length(imuData.timestamps);
numMeasurements = numImuMeasurements + numImageMeasurements;

allTimestamps = [imageTimestamps imuData.timestamps];
[~,measIdsTimeSorted] = sort(allTimestamps); %Sort timestamps in ascending order
 

camMeasId = 0;
imuMeasId = 0;



%Initialize the state
xCorrected = xInit;
xDeadReckon = xInit;

%Initialize the history



%Initialize the history
R_wimu = rotmat_from_quat(xCorrected.q);
R_imuw = R_wimu';
p_imuw_w = xCorrected.p;
T_wimu_estimated = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
T_wcam_estimated = T_wimu_estimated*inv(T_camimu);
T_wimu_gtsam = [];


iter = 1;

%Keep track of landmarks that we see and insert into ISAM2
observedLandmarks = {};
depthFilterSeeds = {};
initializedLandmarkIds = [];


for measId = measIdsTimeSorted
    % Which type of measurement is this?
    if measId > numImageMeasurements
        measType = 'IMU';
        imuMeasId = measId - numImageMeasurements;
    else 
        measType = 'Cam';
        camMeasId = measId;
        %continue;
    end
    
    
    % IMU Measurement
    % ==========================================================
    if strcmp(measType, 'IMU')
        if pipelineOptions.verbose
            disp(['Processing IMU Measurement. ID: ' num2str(imuMeasId)]); 
        end
        
        
        %Calculate dt
        if imuMeasId ~= numImuMeasurements
            dt = imuData.timestamps(imuMeasId+1) - imuData.timestamps(imuMeasId);
        end
        
        
        %Extract the measurements
        imuAccel = imuData.measAccel(:, imuMeasId);
        imuOmega = imuData.measOmega(:, imuMeasId);         
          

        %Predict the next state
        [xCorrected] = integrateIMU(xCorrected, imuAccel, imuOmega, dt, noiseParams, g_w);
        [xDeadReckon] = integrateIMU(xDeadReckon, imuAccel, imuOmega, dt, noiseParams, g_w);


        %=======GTSAM=========
        %Integrate each measurement
        currentSummarizedMeasurement.integrateMeasurement(imuAccel, imuOmega, dt);
        %totalSummarizedMeasurement.integrateMeasurement([imuAccel(1); imuAccel(3); imuAccel(2)], imuOmega, dt);
        %=====================
        
        %Formulate matrices 
        R_wimu = rotmat_from_quat(xDeadReckon.q);
        p_imuw_w = xDeadReckon.p;
        
        
        %Keep track of the state
        T_wimu_estimated(:,:, end+1) = [R_wimu p_imuw_w; 0 0 0 1];
        
   
    % Camera Measurement 
    % ==========================================================
    elseif strcmp(measType, 'Cam')
        if pipelineOptions.verbose
            disp(['Processing Camera Measurement. ID: ' num2str(camMeasId)]); 
        end
        
        
        
        
        %Extract features (fake ones)
        largeInt = 1329329;
        keyPointPixels = imageMeasurements(camMeasId).pixelMeasurements;
        keyPointIds = imageMeasurements(camMeasId).landmarkIds;

        
        %The last IMU state based on integration (relative to the world)
        T_wimu_int = [rotmat_from_quat(xCorrected.q) xCorrected.p; 0 0 0 1];

        
        %If it's the first image, set the current pose to the initial
        %keyFramePose
        if camMeasId == 1
           referencePose.allKeyPointPixels = keyPointPixels;
           referencePose.T_wimu_int = T_wimu_int;
           referencePose.T_wimu_opt = T_wimu_int;
           referencePose.T_wcam_opt = T_wimu_int*inv(T_camimu);
           referencePose.allLandmarkIds = keyPointIds;
           
          
            % =========== GTSAM ============
            % Initialization
            currentPoseKey = symbol('x',1);
            currentVelKey =  symbol('v',1);
            currentBiasKey = symbol('b',1);

            %Initialize the state
            newValues.insert(currentPoseKey, currentPoseGlobal);
             newValues.insert(currentVelKey, currentVelocityGlobal);
            newValues.insert(currentBiasKey, currentBias);
            
            %Add constraints
            %newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
            newFactors.add(NonlinearEqualityPose3(currentPoseKey, currentPoseGlobal));
            newFactors.add(NonlinearEqualityLieVector(currentVelKey, currentVelocityGlobal));
             newFactors.add(NonlinearEqualityConstantBias(currentBiasKey, currentBias));
            
            %Prepare for IMU Integration
            currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
                      currentBias, diag(noiseParams.sigma_a.^2), ...
                      diag(noiseParams.sigma_g.^2), 1e-16 * eye(3));
                
            %Note: We cannot add landmark observations just yet because we
            %cannot be sure that all landmarks will be observed from the
            %next pose (if they are not, the system is underconstrained and  ill-posed)
           
            % ==============================
           
        else
            %The reference pose is either the last keyFrame or the initial pose
            %depending on whether we are initialized or not
            %Caculate the rotation matrix prior (relative to the last keyFrame or initial pose)]
            %currentSummarizedMeasurement
              
              %The odometry change  
              %T_rimu = inv(referencePose.T_wimu_opt)*T_wimu_int;
              
              T_rimu = inv(referencePose.T_wimu_opt)*T_wimu_int;
              
              T_rcam = T_camimu*T_rimu*inv(T_camimu);
              R_rcam = T_rcam(1:3,1:3);
              p_camr_r = homo2cart(T_rcam*[0 0 0 1]');

             

           %Figure out the best feature matches between the current and
           %keyFramePose frame (i.e. 'relative')
           matchedRelIndices = simMatchFeatures(referencePose.allLandmarkIds, keyPointIds);
           
           KLOldkeyPointPixels = referencePose.allKeyPointPixels(:, matchedRelIndices(:,1));
           KLNewkeyPointPixels = keyPointPixels(:, matchedRelIndices(:,2));
           
           %Recalculate the unit vectors
            matchedReferenceUnitVectors = normalize(invK*cart2homo(KLOldkeyPointPixels));
            matchedCurrentUnitVectors = normalize(invK*cart2homo(KLNewkeyPointPixels));
      
           %matchedRefGTPoints = referencePose.landmarksGT_r(:, matchedRelIndices(:,1));
           %matchedCurrGTPoints = imageMeasurements(camMeasId).landmark_c(:, matchedRelIndices(:,2));
           matchedKeyPointIds = keyPointIds(matchedRelIndices(:,2), :);

         
           
           
           %=======DO WE NEED A NEW KEYFRAME?=============
           %Calculate disparity between the current frame the last keyFramePose
           disparityMeasure = calcDisparity(KLOldkeyPointPixels, KLNewkeyPointPixels, R_rcam, K);
           disp(['Disparity Measure: ' num2str(disparityMeasure)]);
           
           
          if (disparityMeasure > pipelineOptions.kfDisparityThreshold)
              


                disp(['Creating new keyframe: ' num2str(keyFrame_i)]);   

                     %=========== GTSAM ===========
        
        % At each non=IMU measurement we initialize a new node in the graph
          currentPoseKey = symbol('x',keyFrame_i+1);
          currentVelKey =  symbol('v',keyFrame_i+1);
          currentBiasKey = symbol('b',keyFrame_i+1);
  
             %Important, we keep track of the optimized state and 'compose'
      %odometry onto it!
      currPose = Pose3(referencePose.T_wimu_opt*T_rimu);
   
             % Summarize IMU data between the previous GPS measurement and now
               newFactors.add(ImuFactor( ...
       currentPoseKey-1, currentVelKey-1, ...
       currentPoseKey, currentVelKey, ...
      currentBiasKey, currentSummarizedMeasurement, g_w, w_coriolis));
  
  %Prepare for IMU Integration
            currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
                      currentBias, diag(noiseParams.sigma_a.^2), ...
                      diag(noiseParams.sigma_g.^2), 1e-16 * eye(3));
             
             
       newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), noiseModel.Diagonal.Sigmas(sqrt(40) * sigma_between_b)));

       if ~initiliazationComplete
           currentVelocityGlobal = LieVector(xCorrected.v);
       end
       
    newValues.insert(currentPoseKey, currPose);
     newValues.insert(currentVelKey, currentVelocityGlobal);
     newValues.insert(currentBiasKey, currentBias);
    
        %=============================
        
        
  
        
                
              inlierIdx = findInliers(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r, KLNewkeyPointPixels, K, pipelineOptions);
%               if size(KLNewkeyPointPixels,2) > 3
%                   [~, ~, newInlierPixels] = estimateGeometricTransform(KLOldkeyPointPixels', KLNewkeyPointPixels', 'similarity', 'MaxDistance', 1);
%                   inlierIdx = find(ismember(KLNewkeyPointPixels',newInlierPixels, 'Rows')');
%                else
%                   inlierIdx = [];
%               end
%              
              %inlierIdx = 1: size(KLNewkeyPointPixels,2);
              %inlierIdx = [];
              printf('%d inliers out of a total of %d matched keypoints', length(inlierIdx), size(KLOldkeyPointPixels,2));

             matchedKeyPointIds = matchedKeyPointIds(inlierIdx, :); 
              matchedReferenceUnitVectors = matchedReferenceUnitVectors(:, inlierIdx);
              matchedCurrentUnitVectors = matchedCurrentUnitVectors(:, inlierIdx);
              
%                triangPoints_r = triangulate(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r); 
%                triangPoints_w = homo2cart(referencePose.T_wcam_opt*cart2homo(triangPoints_r));
%             
               
               
              %Extract the raw pixel measurements
               matchedKeyPointsPixels = KLNewkeyPointPixels(:, inlierIdx);
               matchedRefKeyPointsPixels = KLOldkeyPointPixels(:, inlierIdx);

                    printf(['--------- \n Matched ' num2str(length(inlierIdx)) ' old landmarks. ---------\n']);

                             %=========GTSAM==========
                %Extract intrinsics
                f_x = K(1,1);
                f_y = K(2,2);
                c_x = K(1,3);
                c_y = K(2,3);

                % Create realistic calibration and measurement noise model
                % format: fx fy skew cx cy baseline
                K_GTSAM = Cal3_S2(f_x, f_y, 0, c_x, c_y);
                if pipelineOptions.useRobustMEst
                    mono_model_n_robust = noiseModel.Robust(noiseModel.mEstimator.Huber(pipelineOptions.mEstWeight), noiseModel.Isotropic.Sigma(2, pipelineOptions.obsNoiseSigma));
                else
                    mono_model_n_robust = noiseModel.Isotropic.Sigma(2, pipelineOptions.obsNoiseSigma);
                end

                                  
                 
              
               %====== NORMAL ISAM OPERATION =====
               
               %For all initialized landmarks, add their observations
               obsFromInitializedIds = intersect(matchedKeyPointIds, initializedLandmarkIds);
                   
                    %Add all observation of the initialized landmarks
                      addObsNum = 0; 
                      totalReproError = 0;
                     for kp_i = 1:length(obsFromInitializedIds)
                         kptId = obsFromInitializedIds(kp_i);
                         reprojectionError = calcReprojectionError(newValues.at(currentPoseKey).matrix, matchedKeyPointsPixels(:,matchedKeyPointIds==kptId), isamCurrentEstimate.at(kptId).vector, K, T_camimu);
                         if reprojectionError < 100
                             addObsNum = addObsNum + 1;
                             totalReproError = totalReproError + reprojectionError;
                             newFactors.add(GenericProjectionFactorCal3_S2(Point2(matchedKeyPointsPixels(:,matchedKeyPointIds==kptId)), noiseModel.Isotropic.Sigma(2, 10), currentPoseKey, kptId, K_GTSAM,  Pose3(inv(T_camimu))));
                         end
                     end
                     printf('Added %d new observations (Mean Error: %.5f)', addObsNum, totalReproError/addObsNum);
                 
               
                     %For all others, maintain the depth filter
                     %First remove all observations that have already been
                     %processed
                     unintializedIdx = ~ismember(matchedKeyPointIds, obsFromInitializedIds);
                     matchedKeyPointIds = matchedKeyPointIds(unintializedIdx, :); 
                     matchedReferenceUnitVectors = matchedReferenceUnitVectors(:, unintializedIdx);
                     matchedCurrentUnitVectors = matchedCurrentUnitVectors(:, unintializedIdx);
                     matchedKeyPointsPixels = matchedKeyPointsPixels(:, unintializedIdx);
                     matchedRefKeyPointsPixels = matchedRefKeyPointsPixels(:, unintializedIdx);
          
               

                    %Compute the mean feature depths
                    featureDepths = computeDepths(inv(T_rcam), matchedReferenceUnitVectors, matchedCurrentUnitVectors);
                    meanDepth = mean(featureDepths);
                    
                    
                    T_camw = inv(currPose.matrix*inv(T_camimu));
                    landmarks_c = homo2cart(T_camw*cart2homo(landmarks_w));
                        
                    
                    %Initialize all seeds that are new observations
                    if ~isempty(observedLandmarks)
                        newObsIds = setdiff(matchedKeyPointIds, [observedLandmarks(:).id]);
                    else
                        newObsIds = matchedKeyPointIds;
                    end
                    for kpt_j = 1:length(newObsIds)
                        
                        oli = length(observedLandmarks) + 1;
                        observedLandmarks(oli).id = newObsIds(kpt_j);
                        observedLandmarks(oli).poseKeys = [(currentPoseKey-1) currentPoseKey];
                        %observedLandmarks(oli).pixelObs = [matchedRefKeyPointsPixels(:,matchedKeyPointIds==newObsIds(kpt_j)), matchedKeyPointsPixels(:,matchedKeyPointIds==newObsIds(kpt_j))];
                        observedLandmarks(oli).featVec = matchedReferenceUnitVectors(:,matchedKeyPointIds==newObsIds(kpt_j));
                        observedLandmarks(oli).simpleTriang = triangulate2(matchedReferenceUnitVectors(:,matchedKeyPointIds==newObsIds(kpt_j)), matchedCurrentUnitVectors(:,matchedKeyPointIds==newObsIds(kpt_j)), T_rcam(1:3,1:3), T_rcam(1:3,4));

                        dfi = length(depthFilterSeeds) + 1;
                        
                        depthFilterSeeds(dfi).mu = 1/meanDepth;
                        depthFilterSeeds(dfi).sigma2 = 4/36;
                        depthFilterSeeds(dfi).a = 10;
                        depthFilterSeeds(dfi).b = 10;
                        depthFilterSeeds(dfi).z_range = 2;
                        depthFilterSeeds(dfi).id = newObsIds(kpt_j);
                        depthFilterSeeds(dfi).trueDepth = norm(landmarks_c(:,newObsIds(kpt_j)));
                        depthFilterSeeds(dfi).numObs = 1;
                        depthFilterSeeds(dfi).lastPoseKey = (currentPoseKey-1);
                    end


                      %Update all uninitialized keypoints
                      %uninitializedIds = setdiff(matchedKeyPointIds, initializedLandmarkIds);
    
                        for obs_i = 1:length(matchedKeyPointIds)
                            %Compute depth uncertainties
                            kptId = matchedKeyPointIds(obs_i);
                            currentSeedMask = [depthFilterSeeds(:).id] == kptId;
                            if sum(currentSeedMask) == 0
                                continue;
                            end
                                
                            ol = observedLandmarks([observedLandmarks(:).id] == kptId);
                            firstSeenPoseKey = ol.poseKeys(1);
                            firstSeenFeatVec = ol.featVec;
    
                            %Keep track of observations
                            %observedLandmarks([observedLandmarks(:).id] == kptId).poseKeys = [ol.poseKeys currentPoseKey];
                            %observedLandmarks([observedLandmarks(:).id] == kptId).pixelObs = [ol.pixelObs matchedKeyPointsPixels(:,obs_i)];

                            %Check if we havent updated the state yet
                            %(happens on 2nd image)
                            if keyFrame_i == 1
                                T_camr_feat = T_camw*newValues.at(firstSeenPoseKey).matrix*inv(T_camimu);
                            else
                                T_camr_feat = T_camw*isamCurrentEstimate.at(firstSeenPoseKey).matrix*inv(T_camimu);
                            end
                            
                            measFeatureDepth = computeDepths(T_camr_feat, firstSeenFeatVec, matchedCurrentUnitVectors(:, obs_i));


                            tau = computeTaus(T_camr_feat, matchedCurrentUnitVectors(:, obs_i),measFeatureDepth, K);
                            %tau = 1;
                            invTau = 0.5 * (1.0/max(0.0000001, measFeatureDepth-tau) - 1.0/(measFeatureDepth+tau));

                            %Update seeds with observations from the current
                            %image


                            depthFilterSeeds(currentSeedMask) = updateSeeds(depthFilterSeeds(currentSeedMask), 1/measFeatureDepth, invTau^2);
                        end
                    
                    %Delete all observations that don't have a current
                    %observation
                    %expiredSeedIdx = [depthFilterSeeds(:).lastPoseKey] < currentPoseKey - 5;
                    %depthFilterSeeds(expiredSeedIdx) = [];



                    
                    %Check for convergence
                    convergedSeedIdx = [depthFilterSeeds(:).sigma2] < (1/500)^2;
                    convergedInvDepth = [depthFilterSeeds(convergedSeedIdx).mu];
                    convergedKptIds = [depthFilterSeeds(convergedSeedIdx).id];
                    convergedTrueDepths = [depthFilterSeeds(convergedSeedIdx).trueDepth];
                        convergedNumObs = [depthFilterSeeds(convergedSeedIdx).numObs];



                    if ~isempty(convergedNumObs)
                    printf('Mean number of observations before convergence: %.5f', mean(convergedNumObs));
                    end
                    
                    %Insert all converged depth estimates as landmarks into
                    %the ISAM filter
                    meanEucError = 0;
                    meanEucErrorTri = 0;
                    meanDepthError = 0;
                    for kpt_i = 1:length(convergedKptIds)
                        
                        
                            kptId = convergedKptIds(kpt_i);

                            
                            invD = convergedInvDepth(kpt_i);
                            ol = observedLandmarks([observedLandmarks(:).id] == kptId);
                            

                            
                            T_wcam = isamCurrentEstimate.at(ol.poseKeys(1)).matrix*inv(T_camimu);
                            kptLoc_w = homo2cart(T_wcam*cart2homo((1/invD)*ol.featVec));
                            kptLoc_w_tri = homo2cart(T_wcam*cart2homo(ol.simpleTriang));
                            meanEucError = meanEucError + norm(kptLoc_w - landmarks_w(:,kptId));
                            meanEucErrorTri = meanEucErrorTri + norm(kptLoc_w_tri - landmarks_w(:,kptId));
                            meanDepthError = meanDepthError + abs(1/invD - convergedTrueDepths(kpt_i));

                            
                            %Insert keypoint into the filter
                            newValues.insert(kptId, Point3(kptLoc_w));
%                             for obs_i = 1:size(ol.pixelObs,2)
%                                 newFactors.add(GenericProjectionFactorCal3_S2(Point2(ol.pixelObs(:, obs_i)), mono_model_n_robust, ol.poseKeys(obs_i), kptId, K_GTSAM,  Pose3(inv(T_camimu))));
%                             end
                            
                            %newFactors = addLandmarkObservations(newFactors, kptId, observedLandmarks);
                            newFactors.add(PriorFactorPoint3(kptId, Point3(kptLoc_w), noiseModel.Isotropic.Sigma(3, 0.25)));
                            initializedLandmarkIds = [initializedLandmarkIds kptId];
                    end
                    
                    printf('Mean Euc. Error for %d landmarks inserted: %.5f', length(convergedKptIds), meanEucError/length(convergedKptIds));
                    printf('Mean Simple Triang Error: %.5f', meanEucErrorTri/length(convergedKptIds));
                    printf('Mean Depth Error: %.5f', meanDepthError/length(convergedKptIds));

                     %Delete all converged seeds
                    depthFilterSeeds(convergedSeedIdx) = [];
                   
                  
                  
                    isam.update(newFactors, newValues);
                    isamCurrentEstimate = isam.calculateEstimate();
                   

                        
                    
                   %Reset the new values
                   newFactors = NonlinearFactorGraph;
                   newValues = Values;
             

               
                currentVelocityGlobal = isamCurrentEstimate.at(currentVelKey);
                currentBias = isamCurrentEstimate.at(currentBiasKey);
                currentPoseGlobal = isamCurrentEstimate.at(currentPoseKey);
                
                currentPoseTemp = currentPoseGlobal.matrix;
                 xCorrected.p = currentPoseTemp(1:3,4); 
                 xCorrected.q = quat_from_rotmat(currentPoseTemp(1:3, 1:3));
                 xCorrected.v =  currentVelocityGlobal.vector; %Note velocity has to be in the reference frame!
                 
                xCorrected.b_a = currentBias.accelerometer;
                xCorrected.b_g = currentBias.gyroscope;
                
                 

                %Plot the results
                p_wimu_w = currentPoseGlobal.translation.vector;
                p_wimu_w_int = T_wimu_estimated(1:3,4, end);
                plot(p_wimu_w(1), p_wimu_w(2), 'g*');
                plot(p_wimu_w_int(1), p_wimu_w_int(2), 'r*');
                %set (gcf(), 'outerposition', [25 800, 560, 470])
                hold on;
                drawnow;
                pause(0.01);
                
                 


               %Save keyframe
               %Each keyframe requires:
               % 1. Absolute rotation and translation information (i.e. pose)
               % 2. Triangulated 3D points and associated descriptor vectors

               keyFrames(keyFrame_i).imuMeasId = imuMeasId+1;
               keyFrames(keyFrame_i).T_wimu_opt = currentPoseGlobal.matrix;
               keyFrames(keyFrame_i).T_wimu_int = T_wimu_int;
               keyFrames(keyFrame_i).T_wcam_opt = currentPoseGlobal.matrix*inv(T_camimu);
               keyFrames(keyFrame_i).landmarkIds = matchedKeyPointIds; %Unique integer associated with a landmark
               keyFrames(keyFrame_i).allKeyPointPixels = keyPointPixels;
               keyFrames(keyFrame_i).allLandmarkIds = keyPointIds;

               

               %Update the reference pose
               referencePose = {};
               referencePose = keyFrames(keyFrame_i);

               
                keyFrame_i = keyFrame_i + 1;
               
               

           end %if meanDisparity
           
           
        end % if camMeasId == 1

    end % strcmp(measType...)
    
    iter = iter + 1;
end % for measId = ...

%Plot the landmark locations
lmError = zeros(1, length(initializedLandmarkIds));
for lm_i = 1:length(initializedLandmarkIds)
    lmGTSAMpos = isamCurrentEstimate.at(initializedLandmarkIds(lm_i)).vector;
    lmGTpos = landmarks_w(:,initializedLandmarkIds(lm_i));
    lmError(lm_i) = norm(lmGTpos - lmGTSAMpos);
end
figure
hist(lmError, 50)

%Output the final estimate
for kf_i = 1:(keyFrame_i-1)
    T_wimu_gtsam(:,:, kf_i) = isamCurrentEstimate.at(symbol('x', kf_i+1)).matrix;
end
end

