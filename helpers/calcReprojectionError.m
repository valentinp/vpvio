function [ error ] = calcReprojectionError(imuPoses, pixelMeasurements, landmarks_w, K, T_camimu)
%CALCREPROJECTIONERROR Calculates total pixel reprojection error
%   Detailed explanation goes here

%Iterate over the poses, project landmarks into the camera plane and
%calculate error

error = 0;
for pose_i = 1:size(imuPoses, 3)
    T_wimu = imuPoses(:,:,pose_i);
    T_camw = T_camimu*inv(T_wimu);
    for lm_i = 1:size(landmarks_w, 2)
        landmark_c = homo2cart(T_camw*cart2homo(landmarks_w(:,lm_i)));
        errorPixelVec = pixelMeasurements(:, lm_i, pose_i) - homo2cart(K*landmark_c);
        error = error + sum(errorPixelVec.^2);
    end
end


end

