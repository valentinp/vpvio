function n_p = getLandmarkNoise(viewableLandmarks, pixelMeasurements, T_wCam_GT, simSetup)

%     if T_wCam_GT(3,3)*T_wCam_GT(1,3) > 0
%         n_p = 5*simSetup.pixelNoiseStd*randn(2,size(viewableLandmarks,2)); %image noise        
%     else
%         n_p = 0.2*simSetup.pixelNoiseStd*randn(2,size(viewableLandmarks,2)); %image noise
%     end

         n_p = simSetup.pixelNoiseStd*randn(2,size(viewableLandmarks,2)); %image noise        
  

end

