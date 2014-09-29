function imageMeasurements = genImageMeasurements(T_wCam_GT, landmarks_w, K, simSetup)
%GENIMAGEMEASUREMENTS

% imageMeasurements:
%           array of imageMeasurement structs:
%           imageMeasurements(i).timestamp
%           imageMeasurements(i).pixelMeasurements (2xN)
%           imageMeasurements(i).landmarkIds (Nx1)
%   setup: [struct]


camRes = simSetup.cameraResolution;
sampleRateFactor = floor(simSetup.imuRate/simSetup.cameraRate);
IMUdT = 1/simSetup.imuRate;

% Extract all viewable measurements
k = 1;

for i = 1:size(T_wCam_GT,3)
    
    if mod(i, sampleRateFactor) == 0 || i == 1
        
        
        landmarks_cam = homo2cart(inv(T_wCam_GT(:,:,i))*cart2homo(landmarks_w));
        
        
        pixels = homo2cart(K*landmarks_cam);
        
        
        viewableLandmarksIdx = pixels(1,:) > 0 & pixels(1,:) < camRes(2) & pixels(2,:) > 0 & pixels(2,:) < camRes(1);
        
        %IMPORTANT: ONLY CHECKING PIXELS IS NOT ENOUGH! NEED TO CHECK ALSO
        %IF LANDMARK IS IN FRONT OF THE CAMERA.
        viewableLandmarksIdx = viewableLandmarksIdx & landmarks_cam(3,:) > 0;
        
        %viewableLandmarksIdx = acosd(landmarks_cam(3,:)./sqrt(landmarks_cam(1,:).^2 + landmarks_cam(2,:).^2 + landmarks_cam(3,:).^2)) < fov_cone_angle & landmarks_cam(3,:) > 0;
        viewableLandmarkIds = find(viewableLandmarksIdx);
        viewableLandmarkIds = viewableLandmarkIds(:);
        
        viewableLandmarks = landmarks_cam(:, viewableLandmarksIdx);
        
        
        %n_p = simSetup.pixelNoiseStd*randn(2,size(viewableLandmarks,2)); %image noise
        pixelMeasurements = pixels(:, viewableLandmarksIdx);
        
        %n_p = getLandmarkNoise(viewableLandmarks, pixelMeasurements, T_wCam_GT(:,:,i),simSetup);
        
        n_p = simSetup.pixelNoiseStd*randn(2,size(pixelMeasurements,2)); %image noise        
        

        imageMeasurements(k).timestamp = IMUdT*(i-1);
        imageMeasurements(k).pixelMeasurements =  pixelMeasurements + n_p;
        imageMeasurements(k).landmark_c =  viewableLandmarks;
        imageMeasurements(k).landmark_w =  landmarks_w;
        
        imageMeasurements(k).landmarkIds = viewableLandmarkIds;
        k = k + 1;
    end
end


end

