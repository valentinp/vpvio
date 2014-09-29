function [T_wcam_estimated,T_wimu_estimated, T_wimu_gtsam, keyFrames] = VIOPipelineV2_GTSAMCrucifixThreeView(K, T_camimu, monoImageData, bagImageData, imuData, pipelineOptions, noiseParams, xInit, g_w)
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
% monoImageData:
%           monoImageData.timestamps: 1xM
%           monoImageData.rectImages: WxHxM
% params:
%           params.INIT_DISPARITY_THRESHOLD
%           params.KF_DISPARITY_THRESHOLD
%           params.MIN_FEATURE_MATCHES

% Import opencv
import cv.*;
import gtsam.*;


%===GTSAM INITIALIATION====%
currentPoseGlobal = Pose3(Rot3(rotmat_from_quat(xInit.q)), Point3(xInit.p)); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector(xInit.v); 
currentBias = imuBias.ConstantBias(noiseParams.init_ba, noiseParams.init_bg);
sigma_between_b = [ noiseParams.sigma_ba; noiseParams.sigma_bg ];
w_coriolis = [0;0;0];
% Solver object
isamParams = ISAM2Params;
%isamParams.setRelinearizeSkip(25);
gnParams = ISAM2GaussNewtonParams;
%gnParams.setWildfireThreshold(0.001);
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

%All measurements are assigned a unique measurement ID based on their
%timestamp
numImageMeasurements = length(monoImageData.timestamps);
numImuMeasurements = length(imuData.timestamps);
numMeasurements = numImuMeasurements + numImageMeasurements;

allTimestamps = [monoImageData.timestamps imuData.timestamps];
[~,measIdsTimeSorted] = sort(allTimestamps); %Sort timestamps in ascending order
 

camMeasId = 0;
imuMeasId = 0;
firstImageProcessed = false;


%Initialize the state
xPrev = xInit;


%Initialize movement state
lastMovingPose = currentPoseGlobal;
lastMovingState = xInit;
keyFrameJustCreated = false;
lastPoseNum = 1;

%Initialize the history
R_wimu = rotmat_from_quat(xPrev.q);
R_imuw = R_wimu';
p_imuw_w = xPrev.p;
T_wimu_estimated = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
T_wcam_estimated = T_wimu_estimated*inv(T_camimu);
T_wimu_gtsam = [];


iter = 1;

%Keep track of landmarks
insertedLandmarkIds = [];
initializedLandmarkIds = [];

initialObservations.pixels = [];
initialObservations.poseKeys = [];
initialObservations.triangPoints = [];
initialObservations.ids =  [];

pastObservations.pixels = [];
pastObservations.poseKeys = [];
pastObservations.triangPoints = [];
pastObservations.ids =  [];
pastObservations.kfIds = [];

matchedFeatImg = [];


for measId = measIdsTimeSorted
    % Which type of measurement is this?
    if measId > numImageMeasurements
        measType = 'IMU';
        imuMeasId = measId - numImageMeasurements;
    else 
        measType = 'Cam';
        camMeasId = measId;
    end
    
    
    % IMU Measurement
    % ==========================================================
    if strcmp(measType, 'IMU')
        if pipelineOptions.verbose
            disp(['Processing IMU Measurement. ID: ' num2str(imuMeasId)]); 
        end
        
        %Calculate dt
        try     
            dt = imuData.timestamps(imuMeasId) - imuData.timestamps(imuMeasId - 1);
        catch
            dt = imuData.timestamps(imuMeasId +1) - imuData.timestamps(imuMeasId);
        end
        
        %Extract the measurements
        imuAccel = imuData.measAccel(:, imuMeasId);% + rotmat_from_quat(imuData.measOrient(:,imuMeasId))'*[0 0 9.805]';
        imuOmega = imuData.measOmega(:, imuMeasId);
        
        %=======GTSAM=========
        currentSummarizedMeasurement.integrateMeasurement(imuAccel, imuOmega, dt);
        %=====================
        
        
        
        %Predict the next state
        [xPrev] = integrateIMU(xPrev, imuAccel, imuOmega, dt, noiseParams, g_w);
        R_wimu = rotmat_from_quat(xPrev.q);
        R_imuw = R_wimu';
        p_imuw_w = xPrev.p;
        

        
        
        
        %Keep track of the state
        T_wimu_estimated(:,:, end+1) = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);

   
    % Camera Measurement 
    % ==========================================================
    elseif strcmp(measType, 'Cam')
        if pipelineOptions.verbose
            disp(['Processing Camera Measurement. ID: ' num2str(camMeasId)]); 
        end
        
                    disp(['Processing Camera Measurement. ID: ' num2str(camMeasId)]); 

        %Get measurement data
        %camMeasId
        %currImage = monoImageData.rectImages(:,:,camMeasId);
         currImage = reshape(bagImageData{camMeasId}.data, 1280, 960)';       
   
        %The last IMU state based on integration (relative to the world)
        T_wimu_int = T_wimu_estimated(:,:, end);

  
             
       %If it's the first camera measurements, we're done. Otherwise
       %continue with pipeline
       largeInt = 10000;
        if firstImageProcessed == false
       
               firstImageProcessed = true;
                %Extract keyPoints
                keyPoints = detectBinnedImageFeatures((currImage), pipelineOptions.featureCount);
                keyPointPixels = keyPoints.Location(:,:)';
                keyPointIds = camMeasId*largeInt + [1:size(keyPointPixels,2)];
                
                %Set the history
                previousImage = currImage;
                KLOldKeyPoints = num2cell(double(keyPointPixels'), 2)';
                KLRefPixels = double(keyPointPixels);
     
        
       %Save data into the referencePose struct
           referencePose.allKeyPointPixels = keyPointPixels;
           referencePose.T_wimu_int = T_wimu_int;
           referencePose.T_wimu_opt = T_wimu_int;
           referencePose.T_wcam_opt = T_wimu_int*inv(T_camimu);
           referencePose.allLandmarkIds = keyPointIds;
           referencePose.currImage = currImage;
            
         %Save the first pose  
           firstPose = referencePose;
       
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
            newFactors.add(NonlinearEqualityPose3(currentPoseKey, currentPoseGlobal));
            newFactors.add(NonlinearEqualityLieVector(currentVelKey, currentVelocityGlobal));
             newFactors.add(NonlinearEqualityConstantBias(currentBiasKey, currentBias));
            
            %Prepare for IMU Integration
            currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
                      currentBias, diag(noiseParams.sigma_a.^2), ...
                      diag(noiseParams.sigma_g.^2), 1e-5 * eye(3));
                
            %Note: We cannot add landmark observations just yet because we
            %cannot be sure that all landmarks will be observed from the
            %next pose (if they are not, the system is underconstrained and  ill-posed)
           
            % ==============================

        else
              %The odometry change  
              %T_rimu = inv(referencePose.T_wimu_int)*T_wimu_int;
              T_rimu = T_wimu_int;
              
              %Important: look at the composition!
              T_rcam = T_camimu*T_rimu*inv(T_camimu);
              R_rcam = T_rcam(1:3,1:3);
              p_camr_r = homo2cart(T_rcam*[0 0 0 1]');
              
            
              printf('Tracking %d keypoints.', length(keyPointIds));
              
            %Use KL-tracker to find locations of new points
            if keyFrameJustCreated
                KLOldKeyPoints = num2cell(double(referencePose.allKeyPointPixels'), 2)';
                KLRefPixels = double(referencePose.allKeyPointPixels);
                keyPointIds = referencePose.allLandmarkIds;
                previousImage = referencePose.currImage;
                keyFrameJustCreated = false;
            else
                keyFrameJustCreated = false;
            end
            
            [KLNewKeyPoints, status, ~] = cv.calcOpticalFlowPyrLK(uint8(previousImage), uint8(currImage), KLOldKeyPoints);
            previousImage = currImage;
            
            
            KLOldkeyPointPixels = cell2mat(KLOldKeyPoints(:))';
            KLNewkeyPointPixels = cell2mat(KLNewKeyPoints(:))';
            
            % Remove any points that have negative coordinates
            if size(KLOldkeyPointPixels,2) > 0
            negCoordIdx = KLNewkeyPointPixels(1,:) < 0 | KLNewkeyPointPixels(2,:) < 0;
            else
            negCoordIdx = [];
            end
            badIdx = negCoordIdx | (status == 0)';
            KLNewkeyPointPixels(:, badIdx) = [];
            keyPointIds(badIdx) = [];
            KLOldKeyPoints =  num2cell(double(KLNewkeyPointPixels'), 2)';
            
            KLRefPixels(:, badIdx) = [];
            
            
         
%              %Recalculate the unit vectors
%             KLOldkeyPointUnitVectors = normalize(invK*cart2homo(KLRefPixels));
%             KLNewkeyPointUnitVectors = normalize(invK*cart2homo(KLNewkeyPointPixels));
%             
%            
%            %Unit bearing vectors for all matched points
%            matchedReferenceUnitVectors = KLOldkeyPointUnitVectors;
%            matchedCurrentUnitVectors =  KLNewkeyPointUnitVectors;
%            

           
           %=======DO WE NEED A NEW KEYFRAME?=============
           %Calculate disparity between the current frame the last keyFramePose
           %disparityMeasure = calcDisparity(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, K);
          disparityMeasure = calcDisparity(KLRefPixels, KLNewkeyPointPixels, R_rcam, K);
          disp(['Disparity Measure: ' num2str(disparityMeasure)]);
         
            if norm(imuData.measOmega(:, imuMeasId)) < 0.05
            dispThresh = pipelineOptions.kfDisparityThreshold;
            else
                dispThresh =  5;
                disp('Reducing disparity threshold due to turning!');
            end
            
             %dispThresh = pipelineOptions.kfDisparityThreshold;

          if (~initiliazationComplete && disparityMeasure > pipelineOptions.initDisparityThreshold)  || (initiliazationComplete && disparityMeasure > dispThresh) %(~initiliazationComplete && norm(p_camr_r) > 1) || (initiliazationComplete && norm(p_camr_r) > 1) %(disparityMeasure > INIT_DISPARITY_THRESHOLD) 

              
                   %disp(['Creating new keyframe: ' num2str(keyFrame_i)]);   

                     %=========== GTSAM ===========
        
        % At each non=IMU measurement we initialize a new node in the graph
   currentPoseKey = symbol('x',lastPoseNum+1);
                  currentVelKey =  symbol('v',lastPoseNum+1);
                  currentBiasKey = symbol('b',lastPoseNum+1);
                  lastPoseNum = lastPoseNum + 1;

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
                      diag(noiseParams.sigma_g.^2), 1e-5 * eye(3));

        
       %Keep track of BIAS      
       newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), noiseModel.Diagonal.Sigmas(sigma_between_b)));

    newValues.insert(currentPoseKey, currPose);
     newValues.insert(currentVelKey, currentVelocityGlobal);
     newValues.insert(currentBiasKey, currentBias);
    
        %=============================
       
               %Feature descriptors 
               %matchedRelFeatures = referencePose.allkeyPointFeatures(matchedRelIndices(:,1), :);
                
              
              %[~, ~, inlierIdx1] = frame2frameRANSAC(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam);
              %inlierIdx2 = findInliers(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r, KLNewkeyPointPixels, K, pipelineOptions);
              
              if size(KLNewkeyPointPixels,2) > 3
                  [~, ~, newInlierPixels] = estimateGeometricTransform(KLRefPixels', KLNewkeyPointPixels', 'similarity', 'MaxDistance', 1);
                  inlierIdx = find(ismember(KLNewkeyPointPixels',newInlierPixels, 'Rows')');
               else
                  inlierIdx = [];
              end
              
              printf('%d inliers out of a total of %d matched keypoints', length(inlierIdx), size(KLRefPixels,2));
              %inlierIdx = intersect(inlierIdx1, inlierIdx2);
              %inlierIdx = inlierIdx2;

              %matchedRelFeatures = matchedRelFeatures(inlierIdx, :); 
%               matchedReferenceUnitVectors = matchedReferenceUnitVectors(:, inlierIdx);
%               matchedCurrentUnitVectors = matchedCurrentUnitVectors(:, inlierIdx);
%                
               %Triangulate features
               %All points are expressed in the reference frame
               %triangPoints_r = triangulate(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r); 
               %triangPoints_w = homo2cart(referencePose.T_wcam_opt*cart2homo(triangPoints_r));
    
               %Extract the raw pixel measurements
               matchedKeyPointsPixels = KLNewkeyPointPixels(:, inlierIdx);
               matchedRefKeyPointsPixels = KLRefPixels(:, inlierIdx);
               matchedKeyPointIds = keyPointIds(inlierIdx);
               
               %printf(['--------- \n Matched ' num2str(length(inlierIdx)) ' old landmarks. ---------\n']);

               
               %Extract more FAST features to keep an constant number
               

               
               if  pipelineOptions.featureCount - length(inlierIdx) > 0
                newkeyPoints = detectBinnedImageFeatures((currImage), pipelineOptions.featureCount - length(inlierIdx));
                newkeyPointPixels = newkeyPoints.Location(:,:)';
                newkeyPointIds = camMeasId*largeInt + [1:size(newkeyPointPixels,2)];
               else
                   newkeyPointPixels = [];
                   newkeyPointIds = [];
               end 
               
               %Show feature tracks if requested
               if pipelineOptions.showFeatureTracks
                    if isempty(matchedFeatImg)
                        matchedFeatFig = figure();
                        matchedFeatImg = showMatchedFeatures(referencePose.currImage,currImage, matchedRefKeyPointsPixels', matchedKeyPointsPixels');
                    else
                        set(0,'CurrentFigure',matchedFeatFig)
                        showMatchedFeatures(referencePose.currImage,currImage, matchedRefKeyPointsPixels', matchedKeyPointsPixels');
                    drawnow;
     
                    end
                end
               
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
                  pointNoise = noiseModel.Isotropic.Sigma(3, pipelineOptions.triangPointSigma); 

                %approxBaseline = norm(p_camr_r);
                %Insert estimate for landmark, calculate
                %uncertainty
%                  pointNoiseMat = calcLandmarkUncertainty(matchedRefKeyPointsPixels(:,kpt_j), matchedKeyPointsPixels(:,kpt_j), eye(4), approxBaseline, K);
%                  pointNoise = noiseModel.Gaussian.Covariance(pointNoiseMat);
                                            
                    
                 
                %====== INITIALIZATION ========
               if ~initiliazationComplete
                      %Add a factor that constrains this pose (necessary for
                    %the the first 2 poses)
                    %newFactors.add(PriorFactorPose3(currentPoseKey, currPose, sigma_init_x));
                    %newFactors.add(NonlinearEqualityPose3(currentPoseKey, currPose));
                    
                    disp('Initialization frame.')
                    
                    %Keep track of all observed landmarks
                    for kpt_j = 1:length(matchedKeyPointIds)
                         if keyFrame_i == 1
                              initialObservations.pixels = [initialObservations.pixels matchedRefKeyPointsPixels(:, kpt_j)];
                              initialObservations.poseKeys = [initialObservations.poseKeys (currentPoseKey-1)];
                              initialObservations.ids =  [initialObservations.ids matchedKeyPointIds(kpt_j)];
                              
                              initialObservations.pixels = [initialObservations.pixels matchedKeyPointsPixels(:, kpt_j)];
                              initialObservations.poseKeys = [initialObservations.poseKeys (currentPoseKey)];
                              initialObservations.ids =  [initialObservations.ids matchedKeyPointIds(kpt_j)];
                         else
                              initialObservations.pixels = [initialObservations.pixels matchedKeyPointsPixels(:, kpt_j)];
                              initialObservations.poseKeys = [initialObservations.poseKeys (currentPoseKey)];
                              initialObservations.ids =  [initialObservations.ids matchedKeyPointIds(kpt_j)];
                         end
                    end
                    
                    

                    
                    
                    if keyFrame_i == 2
                            
                            uniqueInitialLandmarkIds = unique(initialObservations.ids);
                        for id = 1:length(uniqueInitialLandmarkIds)
                            kptId = uniqueInitialLandmarkIds(id);
                            allKptObsPixels = initialObservations.pixels(:, initialObservations.ids==kptId);
                            %Ensure that we have observations in all 4 of
                            %the first frames
                            if size(allKptObsPixels, 2) > keyFrame_i
                                allPoseKeys = initialObservations.poseKeys(:, initialObservations.ids==kptId);
                                imuPoses = [];
                                camMatrices = {};
                                for pose_i = 1:length(allPoseKeys)
                                    P = T_camimu*inv(newValues.at(allPoseKeys(pose_i)).matrix);
                                    imuPoses(:,:,pose_i) = newValues.at(allPoseKeys(pose_i)).matrix;
                                    camMatrices{pose_i} = K*P(1:3,:);
                                end
                              
                                %Triangulate using a fancy 3-view method
                                 kptLocEst = vgg_X_from_xP_nonlin(allKptObsPixels,camMatrices, repmat([1280;960], [1, size(allKptObsPixels,2)]));
                                kptLocEst = homo2cart(kptLocEst);
                                %kptLocEst = tvt_solve_qr(camMatrices, {allKptObsPixels(:,1),allKptObsPixels(:,2), allKptObsPixels(:,3)});
                                
                                if  norm(kptLocEst) < 20
                                    
                                    %reprojectionError = calcReprojectionError(imuPoses, reshape(allKptObsPixels,[2 1 size(allKptObsPixels,2)]), kptLocEst, K, T_camimu);
                                      %nsertedLandmarkIds = [insertedLandmarkIds kptId];
                                      initializedLandmarkIds = [initializedLandmarkIds kptId];
                                     % reprojectionError = calcReprojectionError(imuPoses, reshape(allKptObsPixels,[2 1 size(allKptObsPixels,2)]), kptLocEst, K, T_camimu)

                                        newValues.insert(kptId, Point3(kptLocEst));
                                        newFactors.add(PriorFactorPoint3(kptId, Point3(kptLocEst), pointNoise));
                                          for obs_i = 1:size(allKptObsPixels,2)
                                            newFactors.add(GenericProjectionFactorCal3_S2(Point2(allKptObsPixels(:, obs_i)), mono_model_n_robust, allPoseKeys(obs_i), kptId, K_GTSAM,  Pose3(inv(T_camimu))));
                                          end
                                    end
                            end

                        end
                        
                        initiliazationComplete = true;
                              %Batch optimized
                        batchOptimizer = LevenbergMarquardtOptimizer(newFactors, newValues);
                        batchOptimizer.values
                        batchOptimizer.error
                        fullyOptimizedValues = batchOptimizer.optimizeSafely();
                        batchOptimizer.error
                        batchOptimizer.values
                        batchOptimizer.error

                       
                        isam.update(newFactors, fullyOptimizedValues);
                        isamCurrentEstimate = isam.calculateEstimate();
                        if batchOptimizer.error > 100
                            break;
                        end
                        printf('%d landmarks initialized. Inserting into filter.', length(initializedLandmarkIds));
                        if isempty(initializedLandmarkIds) 
                            disp('ERROR. NO LANDMARKS INITIALIZED.');
                            break;
                        end

                        
                        %Reset the new values
                        newFactors = NonlinearFactorGraph;
                         newValues = Values;
                    end
               else
               %====== END INITIALIZATION ========
               
               %====== NORMAL ISAM OPERATION =====
               
               
                   %Keep track of all observed landmarks
                    for kpt_j = 1:length(matchedKeyPointIds)
                        % If this is the first time, we need to add the
                        % previous keyframe observation as well.
                        
                        
                         if ~ismember(matchedKeyPointIds(kpt_j), pastObservations.ids) && ~ismember(matchedKeyPointIds(kpt_j),initializedLandmarkIds) 
                             
                              pastObservations.pixels = [pastObservations.pixels matchedRefKeyPointsPixels(:, kpt_j)];
                              pastObservations.poseKeys = [pastObservations.poseKeys (currentPoseKey-1)];
                              pastObservations.kfIds = [pastObservations.kfIds (keyFrame_i-1)];
                              pastObservations.triangPoints = [pastObservations.triangPoints triangPoints_w(:,kpt_j)];
                              pastObservations.ids =  [pastObservations.ids matchedKeyPointIds(kpt_j)];
                                     
                         end
                              pastObservations.pixels = [pastObservations.pixels matchedKeyPointsPixels(:, kpt_j)];
                              pastObservations.poseKeys = [pastObservations.poseKeys (currentPoseKey)];
                              pastObservations.kfIds = [pastObservations.kfIds (keyFrame_i)];
                              pastObservations.triangPoints = [pastObservations.triangPoints triangPoints_w(:,kpt_j)];
                              pastObservations.ids =  [pastObservations.ids matchedKeyPointIds(kpt_j)];
                    end
                    
                     %Process all landmarks that have gone out of view OR
                     %if they've been inserted during initialization
                    obsFromInitialized = intersect(pastObservations.ids, initializedLandmarkIds);
                    %printf('%d observed landmarks from initialization', length(obsFromInitialized));

                   
                    %Add all observation of the initialized landmarks
                           addObsNum = 0; 
                          totalReproError = 0;
                      for id = 1:length(obsFromInitialized)
                           kptId = obsFromInitialized(id);
                           allKptObsPixels = pastObservations.pixels(:, pastObservations.ids==kptId);
                           allPoseKeys = pastObservations.poseKeys(:, pastObservations.ids==kptId);
                            


                          for obs_i = 1:size(allKptObsPixels,2)
                                     reprojectionError = calcReprojectionError(newValues.at(allPoseKeys(obs_i)).matrix, allKptObsPixels(:, obs_i), isamCurrentEstimate.at(kptId).vector, K, T_camimu);
                                     if reprojectionError < 25
                                         addObsNum = addObsNum + 1;
                                         totalReproError = totalReproError + reprojectionError;
                                         newFactors.add(GenericProjectionFactorCal3_S2(Point2(allKptObsPixels(:, obs_i)), noiseModel.Isotropic.Sigma(2, 10), allPoseKeys(obs_i), kptId, K_GTSAM,  Pose3(inv(T_camimu))));
                                     end
                          end
                      end
                                printf('Added %d new observations (Mean Error: %.5f)', addObsNum, totalReproError/addObsNum);
                 
                     %Remove all added landmarks from qeueu
                     deleteIdx = ismember(pastObservations.ids, obsFromInitialized);
                    pastObservations.pixels(:,  deleteIdx) = [];
                    pastObservations.poseKeys(deleteIdx) = [];
                    pastObservations.triangPoints(:,  deleteIdx) = [];
                    pastObservations.ids(deleteIdx) = [];
               
               
                    
                    %Add all new ids (if they have more than 2
                    %observations)
                    newIds = pastObservations.ids(~ismember(pastObservations.ids , initializedLandmarkIds));
                    [newIdsUnique,newIdsNumUnique] = count_unique(newIds);
                    obsUninitializedIds = newIdsUnique(newIdsNumUnique > 2);
                    %obsUninitializedIds = [];
                    
                    %printf('%d new landmarks found.', length(obsUninitializedIds));

                    newLandmarks = 0;     
                    %Add all uninitialized landmarks
                    for id = 1:length(obsUninitializedIds)
                        
                            kptId = obsUninitializedIds(id);
                            %allKptTriang = pastObservations.triangPoints(:, pastObservations.ids==kptId);
                            allKptObsPixels = pastObservations.pixels(:, pastObservations.ids==kptId);
                            allPoseKeys = pastObservations.poseKeys(:, pastObservations.ids==kptId);
                            %Triangulate the point by taking the mean of
                            %all observations (starting from the 2nd one
                            %since we can't triangulate right away)
                            imuPoses = [];
                           camMatrices = {};
                           for pose_i = 1:length(allPoseKeys)
                                     if allPoseKeys(pose_i) == currentPoseKey
                                         imuPoses(:,:,pose_i) = currPose.matrix;
                                         P = T_camimu*inv(currPose.matrix);
                                     else
                                        imuPoses(:,:,pose_i) = isamCurrentEstimate.at(allPoseKeys(pose_i)).matrix;
                                        P = T_camimu*inv(isamCurrentEstimate.at(allPoseKeys(pose_i)).matrix);
                                     end
                                     
                                      camMatrices{pose_i} = K*P(1:3,1:4);
                           end
                            kptLocEst = vgg_X_from_xP_nonlin(allKptObsPixels,camMatrices, repmat([1280;960], [1, size(allKptObsPixels,2)]));
                            kptLocEst = homo2cart(kptLocEst);
                            
                            
                           %reprojectionError = calcReprojectionError(imuPoses, reshape(allKptObsPixels,[2 1 size(allKptObsPixels,2)]), kptLocEst, K, T_camimu);

                                      
                            %kptLocEst = [mean(allKptTriang(1,2:end)); mean(allKptTriang(2,2:end)); mean(allKptTriang(3,2:end)) ];
                            %kptLocEst = allKptTriang(:,2);
                            tempValues = Values;
                            tempFactors = NonlinearFactorGraph;
                            
                            tempValues.insert(kptId, Point3(kptLocEst));
                             for obs_i = 1:size(allKptObsPixels,2)
                                tempFactors.add(GenericProjectionFactorCal3_S2(Point2(allKptObsPixels(:, obs_i)), mono_model_n_robust, allPoseKeys(obs_i), kptId, K_GTSAM,  Pose3(inv(T_camimu))));
                             end
                             uniquePoseKeys = unique(allPoseKeys);
                             
                             for pose_i = 1:length(uniquePoseKeys)
                                     if allPoseKeys(pose_i) == currentPoseKey
                                        tempValues.insert(uniquePoseKeys(pose_i), currPose);
                                        tempFactors.add(NonlinearEqualityPose3(uniquePoseKeys(pose_i), currPose));
                                     else
                                        tempValues.insert(uniquePoseKeys(pose_i), isamCurrentEstimate.at(uniquePoseKeys(pose_i)));
                                        tempFactors.add(NonlinearEqualityPose3(uniquePoseKeys(pose_i), isamCurrentEstimate.at(uniquePoseKeys(pose_i))));
                                     end
                             end
                            
                              batchOptimizer = GaussNewtonOptimizer(tempFactors, tempValues);
                               if batchOptimizer.error <  pipelineOptions.maxBatchOptimizerError*2
                                   fullyOptimizedValues = batchOptimizer.optimize();
                               else
                                   break;
                               end
                               kptLoc = fullyOptimizedValues.at(kptId).vector;
                               %batchOptimizer.error
                               %kptId
                               if  batchOptimizer.error < pipelineOptions.maxBatchOptimizerError
                                 %insertedLandmarkIds = [insertedLandmarkIds kptId];
                                 initializedLandmarkIds = [initializedLandmarkIds kptId];
                                 newLandmarks = newLandmarks + 1;
                                %batchOptimizer.error
                                if ~isamCurrentEstimate.exists(kptId)
                                    newValues.insert(kptId, Point3(kptLoc));
                                end
 
                                for obs_i = 1:size(allKptObsPixels,2)
                                    newFactors.add(GenericProjectionFactorCal3_S2(Point2(allKptObsPixels(:, obs_i)), mono_model_n_robust, allPoseKeys(obs_i), kptId, K_GTSAM,  Pose3(inv(T_camimu))));
                                end
                                %newFactors.add(PriorFactorPoint3(kptId, Point3(kptLoc), pointNoise));
                             end
                    end
                    
                    printf('%d new landmarks inserted.', newLandmarks);

                     
                    %Remove all added landmarks from qeueu
                    pastObservations.pixels(:,  ismember(pastObservations.ids, initializedLandmarkIds)) = [];
                    pastObservations.poseKeys(ismember(pastObservations.ids, initializedLandmarkIds)) = [];
                    pastObservations.triangPoints(:,  ismember(pastObservations.ids, initializedLandmarkIds)) = [];
                    pastObservations.ids(ismember(pastObservations.ids, initializedLandmarkIds)) = [];
                    
                    %Do the hard work ISAM!
                    
                    isam.update(newFactors, newValues);
                    isamCurrentEstimate = isam.calculateEstimate();

                        
                    
                   %Reset the new values
                   newFactors = NonlinearFactorGraph;
                   newValues = Values;
             
               %==================================
               end %if initializationComplete

               
               %What is our current estimate of the state?
               if initiliazationComplete
                currentVelocityGlobal = isamCurrentEstimate.at(currentVelKey);
                currentBias = isamCurrentEstimate.at(currentBiasKey);
                currentPoseGlobal = isamCurrentEstimate.at(currentPoseKey);
                
                currentPoseTemp = currentPoseGlobal.matrix;
%                 xPrev.p = currentPoseTemp(1:3,4); 
%                 xPrev.q = quat_from_rotmat(currentPoseTemp(1:3, 1:3));
%                 xPrev.v = currentVelocityGlobal.vector;
                xPrev.p = zeros(3,1); 
                 xPrev.q = [1; zeros(3,1)];
                 xPrev.v = currentPoseTemp(1:3, 1:3)'*currentVelocityGlobal.vector; %Note velocity has to be in the reference frame!
         
                xPrev.b_a = currentBias.accelerometer;
                xPrev.b_g = currentBias.gyroscope;
                
                
               else
                   currentPoseGlobal = currPose;
                   currentVelocityGlobal = LieVector(xPrev.v);
               end
               
         
                
               %Save keyframe
               %Each keyframe requires:
               % 1. Absolute rotation and translation information (i.e. pose)
               % 2. Triangulated 3D points and associated descriptor vectors
        
               keyFrames(keyFrame_i).imuMeasId = size(T_wimu_estimated, 3);
               keyFrames(keyFrame_i).T_wimu_opt = currentPoseGlobal.matrix;
               keyFrames(keyFrame_i).T_wimu_int = T_wimu_int;
               keyFrames(keyFrame_i).T_wcam_opt = currentPoseGlobal.matrix*inv(T_camimu);
               %keyFrames(keyFrame_i).pointCloud = triangPoints_w;
               keyFrames(keyFrame_i).landmarkIds = matchedKeyPointIds; %Unique integer associated with a landmark
               keyFrames(keyFrame_i).allKeyPointPixels = [matchedKeyPointsPixels  newkeyPointPixels];
               keyFrames(keyFrame_i).allLandmarkIds = [matchedKeyPointIds newkeyPointIds];
               keyFrames(keyFrame_i).currImage = currImage;

               %Update the reference pose
               referencePose = {};
               referencePose = keyFrames(keyFrame_i);
               keyFrameJustCreated = true;

               keyFrame_i = keyFrame_i + 1;
               
               
                            printf('Total Landmarks in ISAM2: %d', length(initializedLandmarkIds));
                            allPose = zeros(3, length(initializedLandmarkIds));
                            
                   for i = 1:length(initializedLandmarkIds)
                    allPose(:,i) = isamCurrentEstimate.at(initializedLandmarkIds(i)).vector;
                   end
                    if keyFrame_i == 3
                        figure();
                        landmark_axes = gca; 
                        lmPoints = scatter3(allPose(1,:),allPose(2,:),allPose(3,:), 'b*');
                        hold on;
                    elseif keyFrame_i > 3
                        delete(lmPoints);
                        lmPoints = scatter3(landmark_axes, allPose(1,:),allPose(2,:),allPose(3,:), 'b*');
                        p_wimu_w = currentPoseGlobal.translation.vector;
                        p_wimu_w_int = T_wimu_int(1:3,4);
                        plot3(landmark_axes, p_wimu_w(1), p_wimu_w(2),p_wimu_w(3), 'g*');
                        plot3(landmark_axes, p_wimu_w_int(1), p_wimu_w_int(2),p_wimu_w_int(3), 'r*');
                        drawnow; 
                    end

                
                   
           end %if meanDisparity
           
           
        end % if camMeasId == 1
        
    end % strcmp(measType...)
    
    iter = iter + 1;
end % for measId = ...

%Output the final estimate
for kf_i = 1:(keyFrame_i-1)
    T_wimu_gtsam(:,:, kf_i) = isamCurrentEstimate.at(symbol('x', kf_i+1)).matrix;
end

end

