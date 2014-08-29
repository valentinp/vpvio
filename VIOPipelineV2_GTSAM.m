function [T_wcam_estimated,T_wimu_estimated, T_wimu_gtsam, keyFrames] = VIOPipelineV2_GTSAM(K, T_camimu, monoImageData, imuData, pipelineOptions, noiseParams, xInit, g_w)
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

%==========VO PIPELINE=============
R_camimu = T_camimu(1:3, 1:3); 
%==============================


%===GTSAM INITIALIATION====%
currentPoseGlobal = Pose3(Rot3(), Point3()); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector(xInit.v); 
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
noiseModelGPS = noiseModel.Diagonal.Precisions([ [0;0;0]; [1;1;1] ]);
IMU_metadata.AccelerometerBiasSigma = 0.000167;
IMU_metadata.GyroscopeBiasSigma = 2.91e-006;
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
w_coriolis = [0;0;0];

% Solver object
isamParams = ISAM2Params;
%isamParams.setFactorization('CHOLESKY');
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

% =========== GTSAM ============
currentPoseKey = symbol('x',1);
currentVelKey =  symbol('v',1);
currentBiasKey = symbol('b',1);
newValues.insert(currentPoseKey, currentPoseGlobal);
newValues.insert(currentVelKey, currentVelocityGlobal);
newValues.insert(currentBiasKey, currentBias);
newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
insertedKeyPointIds = [];
      currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
          currentBias, 0.01.^2 * eye(3), ...
          0.000175.^2 * eye(3), 0 * eye(3));
% ==============================


%Initialize the history
R_wimu = rotmat_from_quat(xPrev.q);
R_imuw = R_wimu';
p_imuw_w = xPrev.p;
T_wimu_estimated = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
T_wcam_estimated = T_wimu_estimated*inv(T_camimu);


iter = 1;

%Keep track of landmarks
allLandmarkIds = [];
allLandmarkFeatures = [];
allLandmarkPositions_w = [];


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
        try     
            dt = imuData.timestamps(imuMeasId) - imuData.timestamps(imuMeasId - 1);
        catch
            dt = imuData.timestamps(imuMeasId +1) - imuData.timestamps(imuMeasId);
        end
        
        %Extract the measurements
        imuAccel = imuData.measAccel(:, imuMeasId);
        imuOmega = imuData.measOmega(:, imuMeasId);
        
        %Predict the next state
        
        [xPrev] = integrateIMU(xPrev, imuAccel, imuOmega, dt, noiseParams, g_w);
        R_wimu = rotmat_from_quat(xPrev.q);
        R_imuw = R_wimu';
        p_imuw_w = xPrev.p;
        
        %=======GTSAM=========
        currentSummarizedMeasurement.integrateMeasurement(imuAccel, imuOmega, dt);
        %=====================
        
        
        
        %Keep track of the state
        %Note: we don't propagate the state at the last measurement to
        %align with groundtruth
       if imuMeasId ~= numImuMeasurements
            T_wimu_estimated(:,:, end+1) = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
            T_wcam_estimated(:,:, end+1) = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1])*inv(T_camimu);
       end

   
    % Camera Measurement 
    % ==========================================================
    elseif strcmp(measType, 'Cam')
        if pipelineOptions.verbose
            disp(['Processing Camera Measurement. ID: ' num2str(camMeasId)]); 
        end
        
         if camMeasId == 50
             break;
         end
        if imuMeasId < 2
            continue;
        end
        %Get measurement data
        %camMeasId
        currImage = monoImageData.rectImages(:,:,camMeasId);
                
        %The last IMU state
        T_wimu = T_wimu_estimated(:,:, end);
        T_wcam = T_wcam_estimated(:,:, end);
        

        

            

             
       %If it's the first camera measurements, we're done. Otherwise
       %continue with pipeline
       largeInt = 1329329;
        if firstImageProcessed == false
       
       firstImageProcessed = true;
        %Extract keyPoints
        keyPoints = detectMinEigenFeatures(mat2gray(currImage));
        keyPoints = keyPoints.selectStrongest(pipelineOptions.featureCount);
        keyPointPixels = keyPoints.Location(:,:)';
        keyPointIds = camMeasId*largeInt + [1:size(keyPointPixels,2)];
        
       %Save data into the referencePose struct
       referencePose.allKeyPointPixels = keyPointPixels;
       referencePose.R_wk = T_wcam(1:3,1:3);
       referencePose.T_wk = T_wcam;
       referencePose.currImage = currImage;
       referencePose.allLandmarkIds = keyPointIds;

        else
              
             %Keep track of various transformation matrices
              R_wimu = T_wimu(1:3,1:3);
              p_imuw_w = homo2cart(T_wimu*[0 0 0 1]');
              p_camw_w = homo2cart(T_wcam*[0 0 0 1]');
              R_wcam = R_wimu*R_camimu';
              
                
              T_rcam = inv(referencePose.T_wk)*T_wcam;
              R_rcam = T_rcam(1:3,1:3);
              p_camr_r = homo2cart(T_rcam*[0 0 0 1]');
              
            
            %Use KL-tracker to find locations of new points
            KLOldKeyPoints = num2cell(double(referencePose.allKeyPointPixels'), 2)';
            keyPointIds = referencePose.allLandmarkIds;

            [KLNewKeyPoints, status, ~] = cv.calcOpticalFlowPyrLK(uint8(referencePose.currImage), uint8(currImage), KLOldKeyPoints);
            
            
            KLOldkeyPointPixels = cell2mat(KLOldKeyPoints(:))';
            KLNewkeyPointPixels = cell2mat(KLNewKeyPoints(:))';
           
            % Remove any points that have negative coordinates
            negCoordIdx = KLNewkeyPointPixels(1,:) < 0 | KLNewkeyPointPixels(2,:) < 0;
            badIdx = negCoordIdx | (status == 0)';
            KLNewkeyPointPixels(:, badIdx) = [];
            KLOldkeyPointPixels(:, badIdx) = [];
            keyPointIds(badIdx) = [];
         
             %Recalculate the unit vectors
            KLOldkeyPointUnitVectors = normalize(invK*cart2homo(KLOldkeyPointPixels));
            KLNewkeyPointUnitVectors = normalize(invK*cart2homo(KLNewkeyPointPixels));
            
           
           %Unit bearing vectors for all matched points
           matchedReferenceUnitVectors = KLOldkeyPointUnitVectors;
           matchedCurrentUnitVectors =  KLNewkeyPointUnitVectors;
           
           
           %=======DO WE NEED A NEW KEYFRAME?=============
           %Calculate disparity between the current frame the last keyFramePose
           %disparityMeasure = calcDisparity(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, K);
          disparityMeasure = calcDisparity(KLOldkeyPointPixels, KLNewkeyPointPixels);
          disp(['Disparity Measure: ' num2str(disparityMeasure)]);
           
           
          if (~initiliazationComplete && disparityMeasure > pipelineOptions.initDisparityThreshold)  || (initiliazationComplete && disparityMeasure > pipelineOptions.kfDisparityThreshold) %(~initiliazationComplete && norm(p_camr_r) > 1) || (initiliazationComplete && norm(p_camr_r) > 1) %(disparityMeasure > INIT_DISPARITY_THRESHOLD) 

               %====== INITIALIZATION ========
               if ~initiliazationComplete

                %disp('Initializing first keyframe');   
                %disp(['Moved this much: ' ])
                if keyFrame_i == 1
                    initiliazationComplete = true;
                end

               end
               %====== END INITIALIZATION ========

                disp('Creating new keyframe');  
                
                
                        %=========== GTSAM ===========
        
        % At each non=IMU measurement we initialize a new node in the graph
          currentPoseKey = symbol('x',keyFrame_i+1);
          currentVelKey =  symbol('v',keyFrame_i+1);
          currentBiasKey = symbol('b',keyFrame_i+1);
  
             % Summarize IMU data between the previous GPS measurement and now
              newFactors.add(ImuFactor( ...
      currentPoseKey-1, currentVelKey-1, ...
      currentPoseKey, currentVelKey, ...
      currentBiasKey, currentSummarizedMeasurement, g_w, w_coriolis));
  
        currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
          currentBias, 0.01.^2 * eye(3), ...
          0.000175.^2 * eye(3), 0 * eye(3));

              currPose = Pose3(T_wimu);
             

  newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
      noiseModel.Diagonal.Sigmas(1 * sigma_between_b)));

  if keyFrame_i < 2
      newFactors.add(NonlinearEqualityPose3(currentPoseKey, currPose));
  end
    newValues.insert(currentPoseKey, currPose);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
        %=============================
       
               %Feature descriptors 
               %matchedRelFeatures = referencePose.allkeyPointFeatures(matchedRelIndices(:,1), :);
                
              
              %[~, ~, inlierIdx1] = frame2frameRANSAC(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam);
              inlierIdx2 = findInliers(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r, KLNewkeyPointPixels, K, pipelineOptions);
              
              %inlierIdx = intersect(inlierIdx1, inlierIdx2);
              inlierIdx = inlierIdx2;

              %matchedRelFeatures = matchedRelFeatures(inlierIdx, :); 
              matchedReferenceUnitVectors = matchedReferenceUnitVectors(:, inlierIdx);
              matchedCurrentUnitVectors = matchedCurrentUnitVectors(:, inlierIdx);
               
               %Triangulate features
               %All points are expressed in the reference frame
               
               triangPoints_r = triangulate2(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r); 
               triangPoints_w = homo2cart(referencePose.T_wk*cart2homo(triangPoints_r));
            

               %Extract the raw pixel measurements
               matchedKeyPointsPixels = KLNewkeyPointPixels(:, inlierIdx);
               matchedRefKeyPointsPixels = KLOldkeyPointPixels(:, inlierIdx);
               keyPointIds = keyPointIds(inlierIdx);
               
               printf(['--------- \n Matched ' num2str(length(inlierIdx)) ' old landmarks. ---------\n']);

               
               %Extract more FAST features to keep an constant number
               
               if pipelineOptions.featureCount - length(inlierIdx) > 0
                newkeyPoints = detectMinEigenFeatures(mat2gray(currImage));
                newkeyPoints = newkeyPoints.selectStrongest(pipelineOptions.featureCount - length(inlierIdx));
                newkeyPointPixels = newkeyPoints.Location(:,:)';
                newkeyPointIds = camMeasId*largeInt + [1:size(newkeyPointPixels,2)];
               else
                   newkeyPointPixels = [];
                   newkeyPointIds = [];
               end 
               
               %Show feature tracks if requested
               if keyFrame_i > 0 && pipelineOptions.showFeatureTracks
                    showMatchedFeatures(referencePose.currImage,currImage, matchedRefKeyPointsPixels', matchedKeyPointsPixels');
                    drawnow;
                    pause(0.01);
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
                mono_model_n = noiseModel.Diagonal.Sigmas([0.5,0.5]');

    
              
                pointPriorNoise  = noiseModel.Isotropic.Sigma(3,1);
                if keyFrame_i > 1
                   for kpt_j = 1:min(10000,length(keyPointIds))
                        %Make sure we haven't inserted this id in before
                        if ~ismember(keyPointIds(kpt_j), insertedKeyPointIds)
                            newValues.insert(keyPointIds(kpt_j), Point3(triangPoints_w(:, kpt_j)));
                            
                            insertedKeyPointIds = [insertedKeyPointIds keyPointIds(kpt_j)];
                            %Previous observation
                            newFactors.add(GenericProjectionFactorCal3_S2(Point2(double(matchedRefKeyPointsPixels(:,kpt_j))), mono_model_n, currentPoseKey-1, keyPointIds(kpt_j), K_GTSAM,  Pose3(inv(T_camimu))));
                        end
                        %Current observation
                        newFactors.add(PriorFactorPoint3(keyPointIds(kpt_j), Point3(triangPoints_w(:, kpt_j)), pointPriorNoise));
                        newFactors.add(GenericProjectionFactorCal3_S2(Point2(double(matchedKeyPointsPixels(:,kpt_j))), mono_model_n, currentPoseKey, keyPointIds(kpt_j), K_GTSAM, Pose3(inv(T_camimu))));
                   end
                end
                if keyFrame_i > 2
                %Solve the thing!
                isam.update(newFactors, newValues);
                newFactors = NonlinearFactorGraph;
                newValues = Values;
                currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
                
                currentBias = isam.calculateEstimate(currentBiasKey);
                currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
                disp(currentPoseGlobal.translation())
                T_wimu_gtsam(:,:,end+1) = currentPoseGlobal.matrix;
                else
                    if  keyFrame_i == 1
                        T_wimu_gtsam = T_wimu;
                    else
                        T_wimu_gtsam(:,:,end+1) = T_wimu;
                    end
                end
                %========================
   
               disp(['Triangulated landmarks: ' num2str(size(triangPoints_w,2))])
               

               %Save keyframe
               %Each keyframe requires:
               % 1. Absolute rotation and translation information (i.e. pose)
               % 2. Triangulated 3D points and associated descriptor vectors

               keyFrames(keyFrame_i).imuMeasId = size(T_wcam_estimated,3);
               keyFrames(keyFrame_i).camMeasId = camMeasId;
               keyFrames(keyFrame_i).R_wk = R_wcam;
               keyFrames(keyFrame_i).t_kw_w = p_camw_w;
               keyFrames(keyFrame_i).T_wk = T_wcam;
               keyFrames(keyFrame_i).pointCloud = triangPoints_w;
               
               keyFrames(keyFrame_i).pixelMeasurements = matchedKeyPointsPixels;
               keyFrames(keyFrame_i).refPosePixels = matchedRefKeyPointsPixels;
               
               keyFrames(keyFrame_i).landmarkIds = keyPointIds; %Unique integer associated with a landmark
                
               keyFrames(keyFrame_i).allKeyPointPixels = [matchedKeyPointsPixels  newkeyPointPixels];
               keyFrames(keyFrame_i).allLandmarkIds = [keyPointIds newkeyPointIds];
               keyFrames(keyFrame_i).currImage = currImage;


               %Update the reference pose
               referencePose = {};
               referencePose = keyFrames(keyFrame_i);

               keyFrame_i = keyFrame_i + 1;

           else
            %No new keyframe   
              
               
           end %if meanDisparity
           
           
        end % if camMeasId == 1
        
    end % strcmp(measType...)
    
    iter = iter + 1;
end % for measId = ...

end

