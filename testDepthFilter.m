%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('helpers');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('simulation');
addpath('depth_filter/');
addpath('triangulation/');
if ismac
    addpath('/Users/valentinp/Research/opengv/matlab');
else
    addpath('~/Dropbox/Research/Ubuntu/opengv/matlab');
end
if ismac
    addpath('/Users/valentinp/Research/gtsam_toolbox');
else
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
end
 import gtsam.*;

%% Generate the measurments

%Generate a forward trajectory
i = 1;
for t = 0:0.05:2
T_wCam_GT(:,:,i) = [eye(3) [t 0 0]'; 0 0 0 1];
i=i+1;
end
%Generate the true landmarks
landmarks_w = [];
for x_i = 0:0.1:2
    for y_i = -1:0.1:1
        landmarks_w = [landmarks_w [x_i y_i 5]'];
    end
end

%%
simSetup.pixelNoiseStd = 1; %pixels
simSetup.cameraResolution = [1280, 960]; %pixels
simSetup.imuRate = 10; % Hz
simSetup.cameraRate = 10; % Hz

%Set the camera intrinsics
focalLength = 4*1e-3; %4 mm
pixelWidth = 4.8*1e-6;
% 
K  = [focalLength/pixelWidth 0 640;
    0 focalLength/pixelWidth 480;
    0 0 1];

invK = inv(K);
%Generate image data
disp('Generating image measurements...');

imageMeasurements = genImageMeasurements(T_wCam_GT, landmarks_w, K, simSetup);
%
%visualizeVO([], T_wCam_GT(:,:,1:10:size(T_wCam_GT,3)), zeros(3,1), 'Simulation')

disp('Done generating measurements.');


%% Test Inverse Depth
initializedLandmarkIds = [];
observedLandmarks = {};
depthFilterSeeds = {};
meanEucError = 0;
meanDepthError = 0;
meanEucErrorTri = 0;
for camMeasId = 2:length(imageMeasurements)
   
   %Gather Measurements 
   refKeyPointIds = imageMeasurements(camMeasId-1).landmarkIds;
   keyPointIds = imageMeasurements(camMeasId).landmarkIds;
   
   keyPointPixels = imageMeasurements(camMeasId).pixelMeasurements;
   refKeyPointPixels = imageMeasurements(camMeasId-1).pixelMeasurements;

   
   matchedRelIndices = simMatchFeatures(refKeyPointIds, keyPointIds);
   refKeyPointPixels = refKeyPointPixels(:, matchedRelIndices(:,1));
   keyPointPixels = keyPointPixels(:, matchedRelIndices(:,2));
   
   
   matchedReferenceUnitVectors = normalize(invK*cart2homo(refKeyPointPixels));
   matchedCurrentUnitVectors = normalize(invK*cart2homo(keyPointPixels));
   matchedKeyPointIds = keyPointIds(matchedRelIndices(:,2), :);

   T_rcam = inv(T_wCam_GT(:,:,camMeasId-1))*T_wCam_GT(:,:,camMeasId);
   
  %Use depth filter
  
    %Compute the mean feature depths
     featureDepths = computeDepths(inv(T_rcam), matchedReferenceUnitVectors, matchedCurrentUnitVectors);
     meanDepth = mean(featureDepths);
     obsFromInitializedIds = intersect(matchedKeyPointIds, initializedLandmarkIds);



    T_camw = inv(T_wCam_GT(:,:,camMeasId));
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
        observedLandmarks(oli).poseKey = (camMeasId-1);
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
    end

    
%     landmarks_c(:, matchedKeyPointIds==depthFilterSeeds(1).id)
%     depthFilterSeeds(1).sigma2
%     estD = depthFilterSeeds(1).mu
%     depthFilterSeeds(1).trueDepth
    
    
    uninitializedIds = setdiff(matchedKeyPointIds, initializedLandmarkIds);
    
    for obs_i = 1:length(uninitializedIds)
        %Compute depth uncertainties
        kptId = uninitializedIds(obs_i);
        firstSeenId = observedLandmarks([observedLandmarks(:).id] == kptId).poseKey;
        firstSeenFeatVec = observedLandmarks([observedLandmarks(:).id] == kptId).featVec;
        
        T_camr_feat = inv(T_wCam_GT(:,:,camMeasId))*T_wCam_GT(:,:,firstSeenId);
        measFeatureDepth = computeDepths(T_camr_feat, firstSeenFeatVec, matchedCurrentUnitVectors(:, obs_i));

        
        tau = computeTaus(T_camr_feat, matchedCurrentUnitVectors(:, obs_i),measFeatureDepth, K);
        invTau = 0.5 * (1.0/max(0.0000001, measFeatureDepth-tau) - 1.0/(measFeatureDepth+tau));
        
        %Update seeds with observations from the current
        %image


        currentSeedMask = [depthFilterSeeds(:).id] == kptId;
        depthFilterSeeds(currentSeedMask) = updateSeeds(depthFilterSeeds(currentSeedMask), 1/measFeatureDepth, invTau^2);
    end
    
                        
    %Check for convergence
    convergedSeedIdx = [depthFilterSeeds(:).sigma2] < 0.005^2;
    convergedInvDepth = [depthFilterSeeds(convergedSeedIdx).mu];
    convergedKptIds = [depthFilterSeeds(convergedSeedIdx).id];
    convergedTrueDepths = [depthFilterSeeds(convergedSeedIdx).trueDepth];
    convergedNumObs = [depthFilterSeeds(convergedSeedIdx).numObs];
    

    if ~isempty(convergedNumObs)
    printf('Mean number of observations before convergence: %.5f', mean(convergedNumObs));
    printf('Converged Landmarks: %d/%d', length(convergedNumObs), size(landmarks_w,2));
    
    end
                    
                    
    %Insert all converged depth estimates as landmarks into
    %the ISAM filter
    for kpt_i = 1:length(convergedKptIds)


            kptId = convergedKptIds(kpt_i);
            invD = convergedInvDepth(kpt_i);
            ol = observedLandmarks([observedLandmarks(:).id] == kptId);


            T_wcam = T_wCam_GT(:,:,ol.poseKey);
            kptLoc_w = homo2cart(T_wcam*cart2homo((1/invD)*ol.featVec));
            kptLoc_w_tri = homo2cart(T_wcam*cart2homo(ol.simpleTriang));
            
            meanEucError = meanEucError + norm(kptLoc_w - landmarks_w(:,kptId));
            meanEucErrorTri = meanEucErrorTri + norm(kptLoc_w_tri - landmarks_w(:,kptId));
            
            meanDepthError = meanDepthError + abs(1/invD - convergedTrueDepths(kpt_i));
            
            
           initializedLandmarkIds = [initializedLandmarkIds kptId];
    end
    
    
     %Delete all converged seeds
    depthFilterSeeds(convergedSeedIdx) = [];

end
        
meanEucError = meanEucError/length(initializedLandmarkIds)
meanEucErrorTri = meanEucErrorTri/length(initializedLandmarkIds)
meanDepthError = meanDepthError/length(initializedLandmarkIds)
                  

