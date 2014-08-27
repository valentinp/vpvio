function exportG2ODataPRE(keyFrames, landmarks, K, fileName, g2oOptions, clusteringModel, clusterWeights)
%EXPORTG2ODATA Exports poses and landmarks in the form expected by g2o
addpath('/home/valentin/Research/gpml-matlab-v3.4-2013-11-11');
startup();

fid = fopen(fileName, 'w');

%Extract intrinsics
f_x = K(1,1);
f_y = K(2,2);
c_x = K(1,3);
c_y = K(2,3);


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
singleObsLandmarkIds =  uniques(numUnique < 3);
noObsLandmarkIds = setdiff(landmarks.id, uniques);
badLandmarkIds = union(singleObsLandmarkIds, noObsLandmarkIds);


% figure
% hist(pix_error_all)



%Print the camera parameters
    fprintf(fid, 'PARAMS_CAMERAPARAMETERS 0 %.10f %.10f %.10f 0.1\n',f_x,c_x,c_y);


%landmarks struct:
% landmarks.id %1xN
% landmarks.position %3xN

%Print landmarks
for i = 1:length(landmarks.id)
    if ~ismember(landmarks.id(i),badLandmarkIds)
            fprintf(fid, 'VERTEX_XYZ %d %.10f %.10f %.10f \n',landmarks.id(i),landmarks.position(1,i),landmarks.position(2,i),landmarks.position(3,i));
            if g2oOptions.fixLandmarks
                fprintf(fid, 'FIX %d \n', landmarks.id(i));
            end
    end
end

for i = 1:length(keyFrames)
    kf = keyFrames(i);
    R_wk = kf.R_wk;
    q_wk = quat_from_rotmat(R_wk);
    t_kw_w = kf.t_kw_w;

    %Print poses
    fprintf(fid, 'VERTEX_SE3:EXPMAP %d %.10f %.10f %.10f %.10f %.10f %.10f %.10f \n',i,t_kw_w(1),t_kw_w(2), t_kw_w(3), q_wk(2), q_wk(3), q_wk(4), q_wk(1)); %The last number here is the baseline which does not apply to our monocular case
    if i == length(keyFrames)
        fprintf(fid, 'FIX 1 \n');
        fprintf(fid, 'FIX 2 \n');
    end
    
    if g2oOptions.fixPoses && i > 2
        fprintf(fid, 'FIX %d \n', i);
    end

    
end

%Gaussian Process
    %Training Vectors
    x = clusteringModel.centroids;
    t_train = clusterWeights;
    % %Learn the GP!
    covfunc = @covSEiso; 
    likfunc = @likGauss; 
    hyp2.cov = [0;0];    
    hyp2.lik = log(0.1);
    clusteringModel.hyp2 = minimize(hyp2, @gp, -10, @infExact, [], covfunc, likfunc, x', t_train');
    
    

for i = 1:length(keyFrames)
    %Pose: VERTEX_SE3:QUAT 713 -118.755 192.299 4.98256 0.00258538 -0.0132626 -0.831404 0.1055504 
    %Odometry Observation: EDGE_SE3:QUAT 0 1 4.15448 -0.0665288 0.000389663 -0.0107791 0.00867285 -0.00190021 0.999902 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 4.00073 -0.000375887 0.0691425 3.9997 -8.10017e-05 4.00118 
    %Landmark: VERTEX_TRACKXYZ 0 X Y Z
    %Landmark Observation: EDGE_SE3_TRACKXYZ 0 0 X Y Z 1 1 1 1 1 1
    kf = keyFrames(i);
  
    pixelMeasurements = kf.pixelMeasurements;
    landmarkIds = kf.landmarkIds;
    
    
    %Print odometry observations
    if i <length(keyFrames)
        
        %Get the relative transformation
        kfNext = keyFrames(i+1);
        T_kknext = inv(kf.T_wk)*kfNext.T_wk;
        
        t_kw_w = homo2cart(T_kknext(:,4));
        q_wk = quat_from_rotmat(T_kknext(1:3,1:3));
        
        infoMat = g2oOptions.motionEdgeInfoMat;
        %infoMat = inv(keyFrames(i).RPrev(1:6, 1:6));
        infoString = '';
        for l = 1:6
            for m = l:6
                if l == 1 && m == 1
                    infoString = num2str(infoMat(l,m));
                else
                    infoString = [infoString, ' ', num2str(infoMat(l,m))];
                end
            end
        end
        fprintf(fid, 'EDGE_SE3:EXPMAP %d %d %.10f %.10f %.10f %.10f %.10f %.10f %.10f %s \n',i, i+1,t_kw_w(1),t_kw_w(2), t_kw_w(3), q_wk(2), q_wk(3), q_wk(4), q_wk(1), infoString);
    end
    
    %Print landmark observations
        %infoMat = g2oOptions.obsEdgeInfoMat;

    %UseClustersNo = find(clusterWeights > 20);
    
    
    for j = 1:length(landmarkIds)
                clusterId = getClusterIds(kf.predVectors(:,j), clusteringModel);
                %if ismember(clusterId, UseClustersNo)
                %    infoMat = clusterWeights(clusterId)*g2oOptions.obsEdgeInfoMat;
                %else
                %    infoMat = 0.1*g2oOptions.obsEdgeInfoMat;
                %end
                infoMat = g2oOptions.obsEdgeInfoMat;
                %infoMat = getObsEdgeInfoMat(kf.predVectors(:,j), clusteringModel, clusterWeights);
                landmarkInfoString = [num2str(infoMat(1,1)), ' ', num2str(infoMat(1,2)), ' ', num2str(infoMat(2,2))];
        
            if ~ismember(landmarkIds(j),badLandmarkIds)
                        fprintf(fid, 'EDGE_PROJECT_XYZ2UV:EXPMAP %d %d 0 %.10f %.10f %s \n',landmarkIds(j), i, pixelMeasurements(1,j),pixelMeasurements(2,j), landmarkInfoString);
            end
   end
end


fclose(fid);
end

