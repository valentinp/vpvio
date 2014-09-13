function [predVectors] = computePredVectors( pixelLocations, rgb_orig, imuData)
%EXTRACTPREDICTIONSPACE Extract prediction vectors

%Pre-allocate space for prediction vectors


usePredDims = [1; ... %u
               1; ... %v
               1; ... %h
               1; ... %s
               1; ... %v
               1; ... %x contrast
               1; ... %y contrast
               1; ... % entropy
               1; ... % mag(accel)
               1; ... % mag(omega)
               1; ... % point density
               1;     % blur metric
               ];
predSpaceDim = sum(usePredDims); 
predVectors = zeros(predSpaceDim, size(pixelLocations,2));


%normalize rgb image
hsvImage = rgb2hsv(rgb_orig);

rgb = rgb_orig./max(rgb_orig(:));

imageSize = size(rgb(:,:,1))';

%Blur metric is the same for entire image
blurMetricNum = blurMetric(rgb_orig);                
    

for i = 1:size(pixelLocations, 2)
    pixelLoc = round(pixelLocations(:,i));
    pixelLoc = flipud(pixelLoc);
    predVec = [];
    %Magnitude of linear acceleration and angular velocity
    imuPred = [norm(imuData(1:3))/10; norm(imuData(4:6))/0.2];
    
    %Ensure the location is not at the edge of the image, otherwise return
    %zeros for everything but pixel location
    
    if pixelLoc(1) < size(rgb, 1) && pixelLoc(1)  > 1 && pixelLoc(2) < size(rgb,2)  && pixelLoc(2) > 1
        
        %local colour vector (the mean of the r,g,b values in 9 pixel
        %locations surrounding the keypoint)
%         localColour = [mean2(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 1));  ...
%         mean2(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 2));
%         mean2(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 3));
%         ];
        localColour = [mean2(hsvImage(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 1));  ...
        mean2(hsvImage(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 2));
        mean2(hsvImage(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, 3));
        ];
        
        %local contrast vector
        grayImgSeg = rgb2gray(rgb(pixelLoc(1)-1:pixelLoc(1)+1, pixelLoc(2)-1:pixelLoc(2)+1, :));
        
        localContrast = [grayImgSeg(1, 2) - grayImgSeg(3, 2); ...
            grayImgSeg(2, 1) - grayImgSeg(2, 3);
        ];
        
        %local mean intensity
        localEntropy = entropy(grayImgSeg);
    else
        localColour = zeros(3,1);
        localContrast = zeros(2,1);
        localEntropy = 0;
    end
    
    %Landmark Density radius 
    lmSearchRadius = 5;
    %Now add pixel density if we are at least 5 pixels away from the edge
    lmDensity = sum( pixelLocations(1,:) < pixelLoc(1) + lmSearchRadius & ...
                     pixelLocations(1,:) > pixelLoc(1) - lmSearchRadius & ...
                     pixelLocations(2,:) < pixelLoc(2) + lmSearchRadius & ...
                        pixelLocations(2,:) > pixelLoc(2) - lmSearchRadius);
                    
    
   
        if usePredDims(1)
            predVec = [predVec; pixelLoc(1)/imageSize(1)];
        end
        if usePredDims(2)
            predVec = [predVec; pixelLoc(2)/imageSize(2)];
        end
        if usePredDims(3)
            predVec = [predVec; localColour(1)];
        end
        if usePredDims(4)
            predVec = [predVec; localColour(2)];
        end
        if usePredDims(5)
            predVec = [predVec; localColour(3)];
        end
        if usePredDims(6)
            predVec = [predVec; localContrast(1)];
        end
        if usePredDims(7)
            predVec = [predVec; localContrast(2)];
        end
        if usePredDims(8)
            predVec = [predVec; localEntropy];
        end
        if usePredDims(9)
            predVec = [predVec; imuPred(1)];
        end
        if usePredDims(10)
            predVec = [predVec; imuPred(2)];
        end
        if usePredDims(11)
            predVec = [predVec; lmDensity];
        end
        if usePredDims(12)
            predVec = [predVec; blurMetricNum];
        end
        
        predVectors(:,i) = predVec;
    
    
end

end

