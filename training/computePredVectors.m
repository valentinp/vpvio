function [predVectors] = computePredVectors( pixelLocations, rgb, imuData)
%EXTRACTPREDICTIONSPACE Extract prediction vectors

%Pre-allocate space for prediction vectors
predSpaceDim = 11; %2 u,v; 3 r,g,b average, 2 x,y contrast; 1 intensity; 1 mag(accel); 1 mag(omega); 1 point density


predVectors = zeros(predSpaceDim, size(pixelLocations,2));

%normalize rgb image
rgb = rgb./255.0;
imageSize = size(rgb(:,:,1))';


for i = 1:size(pixelLocations, 2)
    pixelLoc = round(pixelLocations(:,i));
    pixelLoc = flipud(pixelLoc);
    
    %Magnitude of linear acceleration and angular velocity
    imuPred = [norm(imuData(1:3)); norm(imuData(4:6))];
    
    %Ensure the location is not at the edge of the image, otherwise return
    %zeros for everything but pixel location
    
    if pixelLoc(1) < size(rgb, 1) && pixelLoc(1)  > 1 && pixelLoc(2) < size(rgb,2)  && pixelLoc(2) > 1
        
        %local colour vector (the mean of the r,g,b values in 9 pixel
        %locations surrounding the keypoint)
        localColour = [mean2(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 1));  ...
        mean2(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 2));
        mean2(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 3));
        ];
        
        %local contrast vector
        grayImgSeg = rgb2gray(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, :));
        
        localContrast = [grayImgSeg(1, 2) - grayImgSeg(3, 2); ...
            grayImgSeg(2, 1) - grayImgSeg(2, 3);
        ];
        
        %local mean intensity
        localIntensity = mean2(grayImgSeg);
        
        predVectors(1:10,i) = [pixelLoc./imageSize; localColour; localContrast; localIntensity; imuPred];
        
    else
        predVectors(1:10,i) = [pixelLoc./imageSize; 0;0;0;0;0;0; imuPred];
    end
    
    %Landmark Density radius 
    lmSearchRadius = 5;
    %Now add pixel density if we are at least 5 pixels away from the edge
    lmDensity = sum( pixelLocations(1,:) < pixelLoc(1) + lmSearchRadius & ...
                     pixelLocations(1,:) > pixelLoc(1) - lmSearchRadius & ...
                     pixelLocations(2,:) < pixelLoc(2) + lmSearchRadius & ...
                        pixelLocations(2,:) > pixelLoc(2) - lmSearchRadius);
    predVectors(11,i) = lmDensity;
    

    
end

end

