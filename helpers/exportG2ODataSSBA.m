function exportG2ODataSSBA(keyFrames, landmarks, K, fileName)
%EXPORTG2ODATA Exports poses and landmarks in the form expected by g2o
fid = fopen(fileName, 'w');

%Extract intrinsics
f_x = K(1,1);
f_y = K(2,2);
c_x = K(1,3);
c_y = K(2,3);

allLandmarkIds = [];

for i = 1:length(keyFrames)
    kf = keyFrames(i);
    allLandmarkIds = [allLandmarkIds kf.landmarkIds];
end

[uniques,numUnique] = count_unique(allLandmarkIds);
singleObsLandmarkIds =  uniques(numUnique < 2);

%Eliminate all crazy pixel errors
outLierLandmarkIds = [];
for keyFrameNum = 1:length(keyFrames)
for l_id = 1:length(keyFrames(keyFrameNum).landmarkIds)
    landmarkId = keyFrames(keyFrameNum).landmarkIds(l_id);
    T_wk = keyFrames(keyFrameNum).T_wk;
    landmark_pos_w = landmarks.position(:,landmarks.id == landmarkId);
    landmark_pos_k = homo2cart(inv(T_wk)*[landmark_pos_w; 1]);
    pixel_coords = homo2cart(K*landmark_pos_k);
    true_pixel_coords = keyFrames(keyFrameNum).pixelMeasurements(:, l_id);
    pix_error = norm(pixel_coords - true_pixel_coords);
    if pix_error > 5
        if ~ismember(landmarkId, outLierLandmarkIds)
           outLierLandmarkIds = [outLierLandmarkIds landmarkId];
        end
    end
end
end

badLandmarkIds = union(singleObsLandmarkIds, outLierLandmarkIds);

disp(length(singleObsLandmarkIds));
disp(length(outLierLandmarkIds));
disp(length(badLandmarkIds));

num2Dmeas = 0;
for i = 1:length(keyFrames)
        landmarkIds = keyFrames(i).landmarkIds;
        for j = 1:length(landmarkIds)
        if ~ismember(landmarkIds(j), badLandmarkIds)
            num2Dmeas = num2Dmeas + 1;
        end
    end
end
fprintf(fid, '%d %d %d \n', length(landmarks.id) - length(badLandmarkIds), length(keyFrames),num2Dmeas);


%Print the camera parameters
fprintf(fid, '%.10f 0 %.10f %.10f %.10f 0 0 0 0 \n',f_x,c_x,f_y,c_y);


%landmarks struct:
% landmarks.id %1xN
% landmarks.position %3xN

%Print landmarks
for i = 1:length(landmarks.id)
    if ~ismember(landmarks.id(i), badLandmarkIds)
        fprintf(fid, '%d %.10f %.10f %.10f \n',landmarks.id(i),landmarks.position(1,i),landmarks.position(2,i),landmarks.position(3,i));
        %fprintf(fid, 'FIX %d \n', landmarks.id(i));
    end
end

for i = 1:length(keyFrames)
    %Pose: VERTEX_SE3:QUAT 713 -118.755 192.299 4.98256 0.00258538 -0.0132626 -0.831404 0.1055504 
    %Odometry Observation: EDGE_SE3:QUAT 0 1 4.15448 -0.0665288 0.000389663 -0.0107791 0.00867285 -0.00190021 0.999902 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 4.00073 -0.000375887 0.0691425 3.9997 -8.10017e-05 4.00118 
    %Landmark: VERTEX_TRACKXYZ 0 X Y Z
    %Landmark Observation: EDGE_SE3_TRACKXYZ 0 0 X Y Z 1 1 1 1 1 1
    kf = keyFrames(i);
  
    
    R_wk = kf.R_wk;
    T_kw = inv(kf.T_wk);
    q_wk = quat_from_rotmat(R_wk);
    t_kw_w = kf.t_kw_w;

     
    pixelMeasurements = kf.pixelMeasurements;
    landmarkIds = kf.landmarkIds;
    
    T_kw = T_kw(1:3, 1:4);
        for l = 1:3
            for m = 1:4
                if l == 1 && m == 1
                    T_string = num2str(T_kw(l,m));
                else
                    T_string = [T_string, ' ', num2str(T_kw(l,m))];
                end
            end
        end
    
    
    %Print poses
    fprintf(fid, '%d %s \n',i,T_string); 
end

for i = 1:length(keyFrames)
     kf = keyFrames(i);
 pixelMeasurements = kf.pixelMeasurements;
    landmarkIds = kf.landmarkIds;
   
    for j = 1:length(landmarkIds)
        if ~ismember(landmarkIds(j), badLandmarkIds)
            fprintf(fid, '%d %d %.10f %.10f 1 \n',i, landmarkIds(j), pixelMeasurements(1,j),pixelMeasurements(2,j));
        end
    end
end
%Add camera offset (necessary for SE3-XYZ edges
%fprintf(fid, 'PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1 \n');


fclose(fid);
end

