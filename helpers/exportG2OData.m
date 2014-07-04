function exportG2OData(keyFrames, landmarks, K, fileName)
%EXPORTG2ODATA Exports poses and landmarks in the form expected by g2o
fid = fopen(fileName, 'w');

%Extract intrinsics
f_x = K(1,1);
f_y = K(2,2);
c_x = K(1,3);
c_y = K(2,3);


%landmarks struct:
% landmarks.id %1xN
% landmarks.position %3xN

%Print landmarks
for i = 1:length(landmarks.id)
    fprintf(fid, 'VERTEX_XYZ %d %.10f %.10f %.10f \n',landmarks.id(i),landmarks.position(1,i),landmarks.position(2,i),landmarks.position(3,i));
end

for i = 1:length(keyFrames)
    %Pose: VERTEX_SE3:QUAT 713 -118.755 192.299 4.98256 0.00258538 -0.0132626 -0.831404 0.1055504 
    %Odometry Observation: EDGE_SE3:QUAT 0 1 4.15448 -0.0665288 0.000389663 -0.0107791 0.00867285 -0.00190021 0.999902 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 4.00073 -0.000375887 0.0691425 3.9997 -8.10017e-05 4.00118 
    %Landmark: VERTEX_TRACKXYZ 0 X Y Z
    %Landmark Observation: EDGE_SE3_TRACKXYZ 0 0 X Y Z 1 1 1 1 1 1
    kf = keyFrames(i);
  
    R_wk = kf.R_wk;
    q_wk = quat_from_rotmat(R_wk);
    t_kw_w = kf.t_kw_w;
    
    pixelMeasurements = kf.pixelMeasurements;
    landmarkIds = kf.landmarkIds;
    
    
    %Print poses
    fprintf(fid, 'VERTEX_CAM %d %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f  \n',i,t_kw_w(1),t_kw_w(2), t_kw_w(3), q_wk(2), q_wk(3), q_wk(4), q_wk(1),f_x,f_y,c_x,c_y,0.5); %The last number here is the baseline which does not apply to our monocular case
    
    if i == 2
        fprintf(fid, 'FIX 1 \n');
        fprintf(fid, 'FIX 2 \n');
    end
    
    
    
    %Print odometry observations
    if i > 1
        
        %Get the relative transformation
        kfPrev = keyFrames(i-1);
        T_kkprev = inv(kf.T_wk)*kfPrev.T_wk;
        
        t_kw_w = homo2cart(T_kkprev(:,4));
        q_wk = quat_from_rotmat(T_kkprev(1:3,1:3));
        
        infoMat = 100*ones(6,6);
        
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
        fprintf(fid, 'EDGE_CAM %d %d %.10f %.10f %.10f %.10f %.10f %.10f %.10f %s \n',i-1,i,t_kw_w(1),t_kw_w(2), t_kw_w(3), q_wk(2), q_wk(3), q_wk(4), q_wk(1), infoString);
    end
    
    %Print landmark observations (with information matrix eye(3))
        noiseMat = 2*diag(ones(1,2));
        infoMat = inv(noiseMat);
        landmarkInfoString = [num2str(infoMat(1,1)), ' ', num2str(infoMat(1,2)), ' ', num2str(infoMat(2,2))];

    for j = 1:length(landmarkIds)
        fprintf(fid, 'EDGE_PROJECT_P2MC %d %d %.10f %.10f %s \n',landmarkIds(j), i, pixelMeasurements(1,j),pixelMeasurements(2,j), landmarkInfoString);
    end
end
%Add camera offset (necessary for SE3-XYZ edges
%fprintf(fid, 'PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1 \n');


fclose(fid);
end

