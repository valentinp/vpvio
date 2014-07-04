function n_p = getLandmarkNoise(viewableLandmarks, pixelMeasurements, T_wCam_GT, simSetup)

%     if T_wCam_GT(3,3)*T_wCam_GT(1,3) > 0
%         n_p = 5*simSetup.pixelNoiseStd*randn(2,size(viewableLandmarks,2)); %image noise        
%     else
%         n_p = 0.2*simSetup.pixelNoiseStd*randn(2,size(viewableLandmarks,2)); %image noise
%     end
    np =  zeros(2, size(viewableLandmarks,2));
    rightHalfIdx = pixelMeasurements(2,:) > simSetup.cameraResolution(1)/2;
    n_p = 0.1*randn(2,size(viewableLandmarks,2));
    n_p(:,rightHalfIdx)  = 3*randn(2,sum(rightHalfIdx)); %image noise     

end

