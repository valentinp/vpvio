function [predVectors] = computePredVectors( pixelLocations, rgb )
%EXTRACTPREDICTIONSPACE Extract prediction vectors

%Pre-allocate space for prediction vectors
predSpaceDim = 8; %2 u,v; 3 r,g,b average, 2 x,y contrast; 1 intensity

predVectors = zeros(predSpaceDim, size(pixelLocations,2));

%normalize rgb image
rgb = rgb./255.0;
imageSize = size(rgb(:,:,1))';


for i = 1:size(pixelLocations, 2)
    pixelLoc = round(pixelLocations(:,i));
    pixelLoc = flipud(pixelLoc);
    
    if i == 128
        a = 2;
    end
    
    %Ensure the location is not at the edge of the image, otherwise return
    %zeros for everything but pixel location
    
    if pixelLoc(1) < size(rgb, 1) && pixelLoc(2) < size(rgb,2)
        
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
        
        predVectors(:,i) = [pixelLoc./imageSize; localColour; localContrast; localIntensity];
        
    else
        predVectors(:,i) = [pixelLoc./imageSize; 0;0;0;0;0;0];
    end
    
    %Feature vector
    
    
end

end

