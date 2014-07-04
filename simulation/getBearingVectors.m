function [bearingVectors] = getBearingVectors( points_w, T_cw, noise)
%GETBEARINGVECTORS Get the bearing vectors (i.e. observations) given a
%point cloud
cameraPoints = homo2cart(T_cw*cart2homo(points_w));
bearingVectors = normalize(cameraPoints);
bearingVectors = addNoise(bearingVectors,1000,noise);
end
