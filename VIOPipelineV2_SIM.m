function [T_wcam_estimated,T_wimu_estimated, keyFrames] = VIOPipelineV2_SIM(K, T_camimu, imageMeasurements, imuData, pipelineOptions, noiseParams, xInit, g_w, g2oOptions)
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


%==========VO PIPELINE=============
R_camimu = T_camimu(1:3, 1:3); 



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

%Keep track of landmarks
allLandmarkIds = [];
allLandmarkFeatures = [];
allLandmarkPositions_w = [];


%Initialize the state
xPrev = xInit;

%Initialize the history

R_wimu = rotmat_from_quat(xPrev.q);
R_imuw = R_wimu';
p_imuw_w = xPrev.p;
T_wimu_estimated = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
T_wcam_estimated = T_wimu_estimated*inv(T_camimu);


iter = 1;
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
        [xPrev] = integrateIMU(xPrev, imuAccel, imuOmega, dt, noiseParams, g_w);
        

        
        %Formulate matrices 
        R_wimu = rotmat_from_quat(xPrev.q);
        R_imuw = R_wimu';
        p_imuw_w = xPrev.p;
        
        
        %Keep track of the state
        T_wimu_estimated(:,:, end+1) = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
        T_wcam_estimated(:,:, end+1) = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1])*inv(T_camimu);
        
   
    % Camera Measurement 
    % ==========================================================
    elseif strcmp(measType, 'Cam')
        if pipelineOptions.verbose
            disp(['Processing Camera Measurement. ID: ' num2str(camMeasId)]); 
        end
        
        
        %Extract features (fake ones)
        surfPoints = imageMeasurements(camMeasId).pixelMeasurements;
        surfFeatures = imageMeasurements(camMeasId).landmarkIds;

        %Back project features onto UNIT sphere
        pixel_locations = surfPoints;
        points_f = normalize(invK*cart2homo(pixel_locations));

        
        %The last IMU state
        T_wimu = T_wimu_estimated(:,:, end);
        T_wcam = T_wcam_estimated(:,:, end);

        
        %If it's the first image, set the current pose to the initial
        %keyFramePose
        if camMeasId == 1
           referencePose.allSurfFeatures = surfFeatures;
           referencePose.allBearingVectors = points_f;
           referencePose.allSurfPoints = surfPoints;
           referencePose.landmarksGT_r =  imageMeasurements(camMeasId).landmark_c;
           referencePose.R_wk = T_wcam(1:3,1:3);
           referencePose.T_wk = T_wcam;
        else
            %The reference pose is either the last keyFrame or the initial pose
            %depending on whether we are initialized or not
            %Caculate the rotation matrix prior (relative to the last keyFrame or initial pose)]
            
            
              R_wimu = T_wimu(1:3,1:3);
              p_imuw_w = homo2cart(T_wimu*[0 0 0 1]');
              p_camw_w = homo2cart(T_wcam*[0 0 0 1]');
              R_wcam = R_wimu*R_camimu';
              
                
              T_rcam = inv(referencePose.T_wk)*T_wcam;
             
              
              R_rcam = T_rcam(1:3,1:3);
              p_camr_r = homo2cart(T_rcam*[0 0 0 1]');

             

           %Figure out the best feature matches between the current and
           %keyFramePose frame (i.e. 'relative')
           matchedRelIndices = simMatchFeatures(referencePose.allSurfFeatures, surfFeatures);


           %Unit bearing vectors for all matched points
           matchedReferenceUnitVectors = referencePose.allBearingVectors(:, matchedRelIndices(:,1));
           matchedCurrentUnitVectors =  points_f(:, matchedRelIndices(:,2));
           
           %matchedRefGTPoints = referencePose.landmarksGT_r(:, matchedRelIndices(:,1));
           %matchedCurrGTPoints = imageMeasurements(camMeasId).landmark_c(:, matchedRelIndices(:,2));
           matchedRelFeatures = surfFeatures(matchedRelIndices(:,2), :);

         
           
           
           %=======DO WE NEED A NEW KEYFRAME?=============
           %Calculate disparity between the current frame the last keyFramePose
            disparityMeasure = calcDisparity(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r, K);
           disp(['Disparity Measure: ' num2str(disparityMeasure)]);
           
           
          if (~initiliazationComplete && disparityMeasure > pipelineOptions.initDisparityThreshold)  || (initiliazationComplete && disparityMeasure > pipelineOptions.kfDisparityThreshold) %(~initiliazationComplete && norm(p_camr_r) > 1) || (initiliazationComplete && norm(p_camr_r) > 1) %(disparityMeasure > INIT_DISPARITY_THRESHOLD) 

              
                %====== INITIALIZATION ========
               if ~initiliazationComplete

                %disp('Initializing first keyframe');   
                %disp(['Moved this much: ' ])
                if keyFrame_i == 3
                    initiliazationComplete = true;
                end

               end
               %====== END INITIALIZATION ========

                disp(['Creating new keyframe: ' num2str(keyFrame_i)]);   

               
               %Magic RANSAC - Deprecated
               %[bestTrans, ~, inlierIdx] = frame2frameRANSAC(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam);
               
                allPixelMeasurements = surfPoints(:,matchedRelIndices(:,2));
                
                 inlierIdx = findInliers(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r, allPixelMeasurements, K, pipelineOptions);
             

                
              matchedRelFeatures = matchedRelFeatures(inlierIdx, :); 
              matchedReferenceUnitVectors = matchedReferenceUnitVectors(:, inlierIdx);
              matchedCurrentUnitVectors = matchedCurrentUnitVectors(:, inlierIdx);
              
%                 t_opt = opengv('twopt',double([1,2]), double(matchedReferenceUnitVectors(:,5:7)), double(matchedCurrentUnitVectors(:,5:7)), double(R_rcam));
% 
%                normalize(t_opt)
%                 normalize(p_camr_r)
               
               %Triangulate features
               %All points are expressed in the reference frame
               
               triangPoints_r = triangulate(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r); 
               triangPoints_c = homo2cart(inv(T_rcam)*cart2homo(triangPoints_r));
               triangPoints_w = homo2cart(referencePose.T_wk*cart2homo(triangPoints_r));
            
               
           
               %Extract the raw pixel measurements
               currPoints = surfPoints(:, matchedRelIndices(inlierIdx,2));
               refPoints = referencePose.allSurfPoints(:, matchedRelIndices(inlierIdx,1));
                refPixels = refPoints;
               
               
               pixelMeasurements = currPoints;
     
                
               %Some random integer to ensure landmark ids do not clash with pose ids in G2O 
               largeInt = 100000;
               
               
               if keyFrame_i == 1
                    landmarkIds = [largeInt:(largeInt + size(pixelMeasurements, 2) - 1)];
                        allLandmarkIds = landmarkIds;
                        allLandmarkFeatures = matchedRelFeatures;
                        allLandmarkPositions_w = triangPoints_w;

               else
                   maxId = max(referencePose.landmarkIds);
                   
                   landmarkIds = [(maxId+1):(maxId+size(pixelMeasurements, 2))];
                   
                   %Check for all landmarks that intersect with the
                    %previous keyframe landmarks
                  %matchedToAllIndices = simMatchFeatures( matchedRelFeatures, allLandmarkFeatures);
                  matchedToAllIndices = simMatchToGlobalLandmarks( matchedRelFeatures, allLandmarkFeatures, allLandmarkPositions_w, pixelMeasurements, K, T_wcam );
   

                       if size(matchedToAllIndices, 1) > 0
                    oldLandmarkIds = allLandmarkIds(matchedToAllIndices(:,2));
                    landmarkIds(matchedToAllIndices(:,1)) = oldLandmarkIds;
                     printf(['--------- \n Found ' num2str(length(landmarkIds) - length(oldLandmarkIds)) ' new landmarks. ---------\n']);
                     printf(['--------- \n Matched ' num2str(length(oldLandmarkIds)) ' old landmarks. ---------\n']);

                       else
                     printf(['--------- \n Found ' num2str(length(landmarkIds)) ' new landmarks. ---------\n']);

                       end

                     [newLandmarkIds,idx] = setdiff(landmarkIds,allLandmarkIds);
                      allLandmarkIds = [allLandmarkIds newLandmarkIds];
                      allLandmarkFeatures = [allLandmarkFeatures; matchedRelFeatures(idx, :)];
                      allLandmarkPositions_w = [allLandmarkPositions_w triangPoints_w(:, idx)];


               end
                


               %Save keyframe
               %Each keyframe requires:
               % 1. Absolute rotation and translation information (i.e. pose)
               % 2. Triangulated 3D points and associated descriptor vectors

               disp(['Triangulated landmarks: ' num2str(size(triangPoints_w,2))])
               keyFrames(keyFrame_i).imuMeasId = size(T_wcam_estimated,3);
               keyFrames(keyFrame_i).R_wk = R_wcam;
               keyFrames(keyFrame_i).t_kw_w = p_camw_w;
               keyFrames(keyFrame_i).T_wk = T_wcam;
               keyFrames(keyFrame_i).pointCloud = triangPoints_w;
               keyFrames(keyFrame_i).pointCloudSurfFeatures = matchedRelFeatures;
               keyFrames(keyFrame_i).pointCloudBearingVectors = matchedCurrentUnitVectors;
               keyFrames(keyFrame_i).allSurfPoints = surfPoints;

               %Ground truth for debugging
               keyFrames(keyFrame_i).landmarksGT_r =  imageMeasurements(camMeasId).landmark_c;

               keyFrames(keyFrame_i).pixelMeasurements = pixelMeasurements;
               keyFrames(keyFrame_i).refPosePixels = refPixels;
               keyFrames(keyFrame_i).landmarkIds = landmarkIds; %Unique integer associated with a landmark
                
               keyFrames(keyFrame_i).allSurfFeatures = surfFeatures;
               keyFrames(keyFrame_i).allBearingVectors = points_f;
               

               %Update the reference pose
               referencePose = {};
               referencePose = keyFrames(keyFrame_i);

               
               
               %New idea! Run the optimization every N keyframes
               if mod(keyFrame_i, pipelineOptions.runOptimizationEveryNKeyframes) == 0
                   landmarks.id = allLandmarkIds;
                   landmarks.position = allLandmarkPositions_w;
                   exportG2ODataExpMap(keyFrames,landmarks, K, 'keyframes.g2o',g2oOptions);
                    %-robustKernel Cauchy -robustKernelWidth 1
                    command2 = '!g2o_bin/g2o -i 30 -v -solver  lm_var -o  opt_keyframes.g2o keyframes.g2o';
                    eval(command2);
                    [T_wc_list_opt, landmarks_w_opt, landmarks_id] = importG2ODataExpMap('opt_keyframes.g2o');
                    allLandmarkPositions_w = landmarks_w_opt;
                    [~, newidx] = ismember(landmarks_id, allLandmarkIds); %New landmarks are a subset of the old ones. Need to only keep relevant features
                    deletedLandmarkIds = setdiff(allLandmarkIds, landmarks_id);
                    allLandmarkFeatures = allLandmarkFeatures(newidx ,:);
                    allLandmarkIds = landmarks_id;

                    
                    
                    for kfi = 1:keyFrame_i
                        keyFrames(kfi).T_wk = T_wc_list_opt(:,:,kfi);
                        keyFrames(kfi).R_wk = T_wc_list_opt(1:3,1:3,kfi);
                        keyFrames(kfi).t_kw_w = T_wc_list_opt(1:3,4,kfi);
                    end
                    
                    %Remove observations of removed landmarks
                    for kfi = 1:keyFrame_i
                        for l_id = 1:length(keyFrames(kfi).landmarkIds)
                           landmarkId = keyFrames(kfi).landmarkIds(l_id);
                           if ismember(landmarkId, deletedLandmarkIds)
                                keyFrames(kfi).landmarkIds(l_id) = NaN;
                                keyFrames(kfi).pixelMeasurements(:,l_id) = [NaN NaN]';
                                keyFrames(kfi).refPosePixels(:,l_id) = [NaN NaN]';
                                keyFrames(kfi).pointCloud(:,l_id) = [NaN NaN NaN]';
                                keyFrames(kfi).pointCloudSurfFeatures(l_id, :) = NaN;
                                keyFrames(kfi).pointCloudBearingVectors(:,l_id) = [NaN NaN NaN]';
                                landmarks.position(:, landmarks.id == landmarkId) = [NaN NaN NaN]';
                                landmarks.id(landmarks.id == landmarkId) = NaN;
                           end
                        end
                         keyFrames(kfi).pixelMeasurements(:, isnan(keyFrames(kfi).pixelMeasurements(1,:))) = [];
                         keyFrames(kfi).refPosePixels(:, isnan(keyFrames(kfi).refPosePixels(1,:))) = [];
                         keyFrames(kfi).pointCloud(:, isnan(keyFrames(kfi).pointCloud(1,:))) = [];
                         keyFrames(kfi).pointCloudSurfFeatures(isnan(keyFrames(kfi).pointCloudSurfFeatures(:,1)), :) = [];
                         keyFrames(kfi).pointCloudBearingVectors(:, isnan(keyFrames(kfi).pointCloudBearingVectors(1,:))) = [];
                         keyFrames(kfi).landmarkIds(isnan(keyFrames(kfi).landmarkIds)) = [];
                    end %for kfi = 1:keyFrame_i
                    
               end %if mod(keyFrame_i, 30) == 0
               
                keyFrame_i = keyFrame_i + 1;

               

           end %if meanDisparity
           
           
        end % if camMeasId == 1

    end % strcmp(measType...)
    
    iter = iter + 1;
end % for measId = ...

end

