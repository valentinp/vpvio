function [T_wc_list_opt, landmarks_w_opt] = processWithGTSAM(keyFrames, landmarks, K, g2oOptions)
import gtsam.*;

%Add observations to the previous keyframe
for i = 2:length(keyFrames)
    prevKf = keyFrames(i-1);
    kf = keyFrames(i);
    [newLandmarkIds,idx] = setdiff(kf.landmarkIds, prevKf.landmarkIds);
     keyFrames(i-1).landmarkIds = [keyFrames(i-1).landmarkIds newLandmarkIds];
     keyFrames(i-1).pixelMeasurements = [keyFrames(i-1).pixelMeasurements kf.refPosePixels(:, idx)];
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
        landmarks.position(:, landmarks.id == landmarkId) = [NaN NaN NaN]';
        landmarks.id(landmarks.id == landmarkId) = NaN;
   end
end

 keyFrames(1).pixelMeasurements(:, isnan(keyFrames(1).pixelMeasurements(1,:))) = [];
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
        keyFrames(kf).landmarkIds(l_ids) = [];
        totalDeletions = totalDeletions + length(l_ids);
    end
end
printf(['--------- \nDeleted ' num2str(totalDeletions) ' bad observations.\n---------\n']);


%Final cleanup: ensure there are no landmarks that have less than 2
%observations

allLandmarkIds = [];
for i = 1:length(keyFrames)
    kf = keyFrames(i);
    allLandmarkIds = [allLandmarkIds kf.landmarkIds];
end
[uniques,numUnique] = count_unique(allLandmarkIds);
singleObsLandmarkIds =  uniques(numUnique < 2);
noObsLandmarkIds = setdiff(landmarks.id, uniques);
badLandmarkIds = union(singleObsLandmarkIds, noObsLandmarkIds);



% Create graph container and add factors to it
graph = NonlinearFactorGraph;

% add a constraint on the starting pose

%Extract intrinsics
f_x = K(1,1);
f_y = K(2,2);
c_x = K(1,3);
c_y = K(2,3);

% Create realistic calibration and measurement noise model
% format: fx fy skew cx cy baseline
K = Cal3_S2(f_x, f_y, 0, c_x, c_y);
mono_model_r = noiseModel.Diagonal.Sigmas([3,3]');
mono_model_l = noiseModel.Diagonal.Sigmas([0.1,0.1]');
mono_model_n = noiseModel.Diagonal.Sigmas([1,1]');


%landmarks struct:
% landmarks.id %1xN
% landmarks.position %3xN



% Create initial estimate for camera poses and landmarks
initialEstimate = Values;



for i = 1:length(keyFrames)
    kf = keyFrames(i);
    R_wk = kf.R_wk;
    t_kw_w = kf.t_kw_w;
    
    pixelMeasurements = kf.pixelMeasurements;
    landmarkIds = kf.landmarkIds;
    
    currPose = Pose3(Rot3(R_wk), Point3(t_kw_w));
    %Fix the first pose 
    if i < 3
         graph.add(NonlinearEqualityPose3(i, currPose));
    end

     
    if i > 1
        delta = prevPose.between(currPose);
        covariance = noiseModel.Diagonal.Sigmas([0.0002; 0.0002; 0.0002; 0.01; 0.01; 0.01]);
        graph.add(BetweenFactorPose3(i-1,i, delta, covariance));
    end
    
    prevPose = currPose;

    for j = 1:length(landmarkIds)
        if ~ismember(landmarkIds(j), badLandmarkIds) 
         if pixelMeasurements(:,2) > 320   
         graph.add(GenericProjectionFactorCal3_S2(Point2(double(pixelMeasurements(:,j))), mono_model_r, i, landmarkIds(j), K));
         else
        graph.add(GenericProjectionFactorCal3_S2(Point2(double(pixelMeasurements(:,j))), mono_model_l, i, landmarkIds(j), K));
         end
         
         end
    end
    
end
for i = 1:length(keyFrames)
    kf = keyFrames(i);
    R_wk = kf.R_wk;
    t_kw_w = kf.t_kw_w;

 %Print poses
    initialEstimate.insert(i, Pose3(Rot3(R_wk), Point3(t_kw_w)));
end  
for i = 1:length(landmarks.id)
            if ~ismember(landmarks.id(i), badLandmarkIds)
                initialEstimate.insert(landmarks.id(i), Point3(double(landmarks.position(:,i))));
            end
end

% optimize
fprintf(1,'Optimizing\n'); tic
params = LevenbergMarquardtParams;
params.setMaxIterations(1000);
params.setVerbosity('ERROR');
params.print('');
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, params);
result = optimizer.optimize();
toc

T_wc_list_opt = zeros(4,4,length(keyFrames));
landmarks_w_opt = zeros(3, length(landmarks.id));

for i = 1:length(keyFrames)
    T_wc_list_opt(:,:,i) = result.at(i).matrix;
end  
landmarks_w_opt = [];
% 
% for i = 1:length(landmarks.id)
%                 if p~ismember(landmarks.id(i), singleObsLandmarkIds)
%     landmarks_w_ot(:,i) = result.at(landmarks.id(i)).vector;
%                 end
% end


end

