function [T_wcam_estimated,T_wimu_estimated, keyFrames, clusterWeights, allFrames] = VIOPipelineV2_PRE(K, T_camimu, monoImageData, rgbImageData, imuData, pipelineOptions, noiseParams, xInit, g_w, clusteringModel, T_wCam_GT)
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
%import cv.*;
opencvDetector = cv.FeatureDetector(pipelineOptions.featureDetector);
opencvMatcher = cv.DescriptorMatcher(pipelineOptions.descriptorMatcher);
opencvExtractor = cv.DescriptorExtractor(pipelineOptions.descriptorExtractor);

        
%==========VO PIPELINE=============
R_camimu = T_camimu(1:3, 1:3); 
%==============================


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


%Initialize the state
xPrev = xInit;
RPrev = 0.01*eye(15);

%Initialize the history
R_wimu = rotmat_from_quat(xPrev.q);
R_imuw = R_wimu';
p_imuw_w = xPrev.p;
T_wimu_estimated = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
T_wcam_estimated = T_wimu_estimated*inv(T_camimu);

iter = 1;
imuDataStruct = {};
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
            %disp('WARNING: Cannot calculate dt. Assuming IMU recording rate of 7.5Hz.');
            %dt = 0.15;
            dt = imuData.timestamps(imuMeasId+1) - imuData.timestamps(imuMeasId);
        end
        
        %Extract the measurements
        imuAccel = imuData.measAccel(:, imuMeasId);
        imuOmega = imuData.measOmega(:, imuMeasId);
        
        %Predict the next state
        
        [xPrev] = integrateIMU(xPrev, imuAccel, imuOmega, dt, noiseParams, g_w);
        [Rprev] = propagateCovariance(xPrev, imuAccel, dt, noiseParams, RPrev);
        
        imuTemp.v = xPrev.v;
        imuTemp.a = imuAccel;
        imuTemp.omega = imuOmega;
        imuTemp.dt = dt;
        imuTemp.g_w = g_w;
        
        imuDataStruct{end+1} = imuTemp;
         R_wimu = rotmat_from_quat(xPrev.q);
        R_imuw = R_wimu';
        p_imuw_w = xPrev.p;
         
        
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
                

        %Get measurement data
        %camMeasId
        currImage = uint8(monoImageData.rectImages(:,:,camMeasId));
        

        %Extract features, select N strongest
        surfPoints = opencvDetector.detect(currImage);
        [~,sortedKeyPointIds] = sort([surfPoints.response]);
        surfPoints = surfPoints(sortedKeyPointIds(1:min(pipelineOptions.featureCount,length(surfPoints))));

        
        
        surfFeatures = opencvExtractor.compute(currImage, surfPoints);
        pixel_locations = reshape([surfPoints.pt], [2 length(surfPoints)]);
        
        
        
        %Necessary for FAST points
        %surfFeatures = surfFeatures.Features;
        

        %Back project features onto UNIT sphere
        %pixel_locations = surfPoints.Location(:,:)';
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
           referencePose.AllPixelMeasurements = pixel_locations;
           referencePose.R_wk = T_wcam(1:3,1:3);
           referencePose.T_wk = T_wcam;
           if imuMeasId == 0
                referencePose.T_wk_gt = T_wCam_GT(:,:, 1);
           else
               referencePose.T_wk_gt = T_wCam_GT(:,:, imuMeasId);
           end
           referencePose.currImage = currImage;
        else
            %The reference pose is either the last keyFrame or the initial pose
            %depending on whether we are initialized or not
            
              R_wimu = T_wimu(1:3,1:3);
              p_imuw_w = homo2cart(T_wimu*[0 0 0 1]');
              p_camw_w = homo2cart(T_wcam*[0 0 0 1]');
              R_wcam = R_wimu*R_camimu';
              
                
              T_rcam = inv(referencePose.T_wk)*T_wcam;
              R_rcam = T_rcam(1:3,1:3);
              p_camr_r = homo2cart(T_rcam*[0 0 0 1]');
              

           %Figure out the best feature matches between the current and
           %keyFramePose frame (i.e. 'relative')
           matches = opencvMatcher.match(referencePose.allSurfFeatures, surfFeatures);
           [~,sortedMatchIds] = find([matches.distance] < pipelineOptions.minMatchDistance);
           matches = matches(sortedMatchIds);
           matchedRelIndices = [[matches.queryIdx]' [matches.trainIdx]'] ;
           matchedRelIndices = matchedRelIndices + ones(size(matchedRelIndices)); %OpenCV is 0-indexed
           
           
           

           %Error checking
           disp(['SURF Matches: ' num2str(size(matchedRelIndices, 1))]);


           %Unit bearing vectors for all matched points
           matchedReferenceUnitVectors = referencePose.allBearingVectors(:, matchedRelIndices(:,1));
           matchedCurrentUnitVectors =  points_f(:, matchedRelIndices(:,2));
           
           %=======DO WE NEED A NEW KEYFRAME?=============
           %Calculate disparity between the current frame the last keyFramePose
           %disparityMeasure = calcDisparity(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, K);
           disparityMeasure = calcDisparity(referencePose.AllPixelMeasurements(:, matchedRelIndices(:,1)), pixel_locations(:, matchedRelIndices(:,2)));
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
       
               %Feature descriptors 
               matchedRelFeatures = surfFeatures(matchedRelIndices(:,2), :);
               allPixelMeasurements = pixel_locations(:, matchedRelIndices(:,2));
                
              
              %[~, ~, inlierIdx1] = frame2frameRANSAC(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam);
             
              %Calculate the predicition vectors
              predVectors = computePredVectors(allPixelMeasurements, rgbImageData.rectImages(:,:,:, camMeasId));
              clusterIds = getClusterIds(predVectors, clusteringModel);
              inlierIdx2 = findInliers(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r, allPixelMeasurements, K, pipelineOptions, clusterIds);
              
              %inlierIdx = intersect(inlierIdx1, inlierIdx2);
              inlierIdx = inlierIdx2;

              
              
              matchedRelFeatures = matchedRelFeatures(inlierIdx, :); 
              matchedReferenceUnitVectors = matchedReferenceUnitVectors(:, inlierIdx);
              matchedCurrentUnitVectors = matchedCurrentUnitVectors(:, inlierIdx);
              
              
              
               %Triangulate features
               %All points are expressed in the reference frame
               
               triangPoints_r = triangulate2(matchedReferenceUnitVectors, matchedCurrentUnitVectors, R_rcam, p_camr_r); 
               triangPoints_w = homo2cart(referencePose.T_wk*cart2homo(triangPoints_r));
            

               %Extract the raw pixel measurements
               currPoints = surfPoints(matchedRelIndices(inlierIdx,2));
               refPoints = referencePose.allSurfPoints(matchedRelIndices(inlierIdx,1));
               refPixels = reshape([refPoints.pt], [2, length(refPoints)])';
              
               pixelMeasurements = allPixelMeasurements(:, inlierIdx);
              
               %Perform feature learning
               %Learn the weights associated with each cluster
               %Each row of clusterWeights represents a single keyframe
              %========================
               
               T_rc_gt = inv(referencePose.T_wk_gt)*T_wCam_GT(:,:, camMeasId);
               predVectors = computePredVectors(pixelMeasurements, rgbImageData.rectImages(:,:,:, camMeasId));

               %pixelMeasurements, rgbimage, T_rc_gt,matchedReferenceUnitVectors, matchedCurrentUnitVectors, T_rcam, clusteringModel
               clusterWeights(keyFrame_i,:) = learnClusterWeights(pixelMeasurements, rgbImageData.rectImages(:,:,:, camMeasId),T_rc_gt,matchedReferenceUnitVectors,matchedCurrentUnitVectors, T_rcam, clusteringModel);
                
               %========================

               
               %Show feature tracks if requested
               if keyFrame_i > 0 && pipelineOptions.showFeatureTracks

                    showMatchedFeatures(referencePose.currImage,currImage, refPixels, pixelMeasurements');
                    hold on;
                    clusterIds = getClusterIds(predVectors, clusteringModel);
                    plot(pixelMeasurements(1, clusterIds == 1), pixelMeasurements(2, clusterIds == 1), 'mo' ,'MarkerSize',10);
                    plot(pixelMeasurements(1, clusterIds == 2), pixelMeasurements(2, clusterIds == 2), 'co', 'MarkerSize',10);
                    drawnow;
                    allFrames(keyFrame_i) = getframe;
                    pause(0.01);
               end
               
               %Keep track of landmarks
               %Some random integer to ensure landmark ids do not clash with pose ids in G2O
               largeInt = 100000;
           
               if size(matchedRelFeatures, 1) > 0 %Ensure there are features!
                   if keyFrame_i == 1
                        landmarkIds = [largeInt:(largeInt + size(pixelMeasurements, 2) - 1)];
                        allLandmarkIds = landmarkIds;
                        allLandmarkFeatures = matchedRelFeatures;
                        allLandmarkPositions_w = triangPoints_w;
                   else
                       maxId = max(referencePose.landmarkIds);

                       if isempty(maxId)
                           maxId = largeInt;
                       end
                       landmarkIds = [(maxId+1):(maxId+size(pixelMeasurements, 2))];

                       %Check for all landmarks that intersect with the
                        %previous keyframe landmarks

                        %Add unique ids
                        
                        
                        matchedToAllIndices = matchToGlobalLandmarks( matchedRelFeatures, allLandmarkFeatures, allLandmarkPositions_w, pixelMeasurements, K, T_wcam );
                        
                        %OpenCV way: NOTE matches are not 1to1!
                        %matches = opencvMatcher.match(allLandmarkFeatures, matchedRelFeatures);
                        %[~,sortedMatchIds] = find([matches.distance] < 0.2);
                        %   matches = matches(sortedMatchIds);
                        %   matchedToAllIndices = [[matches.queryIdx]' [matches.trainIdx]'] ;
                        %   matchedToAllIndices = matchedToAllIndices + ones(size(matchedToAllIndices)); %OpenCV is 0-indexed

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
               end
                
                
   
               disp(['Triangulated landmarks: ' num2str(size(triangPoints_w,2))])
               
               %If there are no points, do not save this keyframe
               if size(triangPoints_w,2) < 1
                continue;
               end

               %Save keyframe
               %Each keyframe requires:
               % 1. Absolute rotation and translation information (i.e. pose)
               % 2. Triangulated 3D points and associated descriptor vectors

               keyFrames(keyFrame_i).imuMeasId = size(T_wcam_estimated,3);
               keyFrames(keyFrame_i).camMeasId = camMeasId;
               
               %Record and reset the covariance
               keyFrames(keyFrame_i).RPrev = Rprev;
               keyFrames(keyFrame_i).imuDataStruct = imuDataStruct;
               imuDataStruct = {};

               %Rprev = zeros(15);
               
               keyFrames(keyFrame_i).R_wk = R_wcam;
               keyFrames(keyFrame_i).t_kw_w = p_camw_w;
               keyFrames(keyFrame_i).T_wk = T_wcam;
               keyFrames(keyFrame_i).T_wk_gt = T_wCam_GT(:,:, imuMeasId);

               keyFrames(keyFrame_i).pointCloud = triangPoints_w;
               keyFrames(keyFrame_i).pointCloudSurfFeatures = matchedRelFeatures;
               keyFrames(keyFrame_i).predVectors = predVectors;
               keyFrames(keyFrame_i).pointCloudBearingVectors = matchedCurrentUnitVectors;
               keyFrames(keyFrame_i).allSurfPoints = surfPoints;

               keyFrames(keyFrame_i).pixelMeasurements = pixelMeasurements;
               keyFrames(keyFrame_i).AllPixelMeasurements = pixel_locations;
               
               keyFrames(keyFrame_i).refPosePixels = refPixels';
               
               keyFrames(keyFrame_i).landmarkIds = landmarkIds; %Unique integer associated with a landmark
                
               keyFrames(keyFrame_i).allSurfFeatures = surfFeatures;
               keyFrames(keyFrame_i).allBearingVectors = points_f;
               
               keyFrames(keyFrame_i).currImage = currImage;


               %Update the reference pose
               referencePose = {};
               referencePose = keyFrames(keyFrame_i);

               keyFrame_i = keyFrame_i + 1;
               
               

           end %if meanDisparity
           
           
        end % if camMeasId == 1

    end % strcmp(measType...)
    
    iter = iter + 1;
end % for measId = ...

end

