function [clusteringModel, inFraction] = clusterData(rgbImageData,monoImageData, imuData, numClusters, Km, featNum)
%CLUSTERDATA Create clusters in the prediction space
allPredVectors = [];

for i = 1:size(monoImageData.rectImages, 3)
    image = uint8(monoImageData.rectImages(:,:,i));
    rgbimage = rgbImageData.rectImages(:,:,:,i);
    imageTimestamp = monoImageData.timestamps(i);
    
    keyPoints = detectFASTFeatures(mat2gray(image));
    keyPoints = keyPoints.selectStrongest(featNum);
    keyPointPixels = keyPoints.Location(:,:)';
    
    %Computes Prediction Space points based on the image and keypoint position
    imu_i = findClosestTimestamp(imageTimestamp, imuData.timestamps);
    imuAccel = imuData.measAccel(:, imu_i);
    imuOmega = imuData.measOmega(:, imu_i);
    imuDataRecord = [imuAccel; imuOmega];
    
    predVectors = computePredVectors(keyPointPixels, rgbimage, imuDataRecord);
    allPredVectors = [allPredVectors predVectors];
end

figure;
subplot(4,3,1)
hist(allPredVectors(1,:))
title('u')
subplot(4,3,2)
hist(allPredVectors(2,:))
title('v')
subplot(4,3,3)
hist(allPredVectors(3,:))
title('h')
subplot(4,3,4)
hist(allPredVectors(4,:))
title('s')
subplot(4,3,5)
hist(allPredVectors(5,:))
title('v')
subplot(4,3,6)
hist(allPredVectors(6,:))
title('x contrast')
subplot(4,3,7)
hist(allPredVectors(7,:))
title('y contrast')
subplot(4,3,8)
hist(allPredVectors(8,:))
title('entropy')
subplot(4,3,9)
hist(allPredVectors(9,:))
title('mag(accel)')
subplot(4,3,10)
hist(allPredVectors(10,:))
title('mag(omega)')
subplot(4,3,11)
hist(allPredVectors(11,:))
title('point density')
subplot(4,3,12)
hist(allPredVectors(12,:))
title('blur metric')

%[C, ptsC, centres] = dbscan(allPredVectors, 10, 100)

[idx, C,~,D] = kmeans(allPredVectors', numClusters);


[COEFF,SCORE, latent] = princomp(allPredVectors');
latent
COEFF


%Determine the mean distance of points within a cluster to the cluster's centroid
%This sets boundaries

meanCentroidDists = zeros(numClusters, 1);
predVectorsWithinBoundaries = 0;
for ic = 1:numClusters
    meanCentroidDists(ic) = Km*mean(D(idx == ic, ic));
    predVectorsWithinBoundaries = predVectorsWithinBoundaries + sum(D(idx == ic, ic) < meanCentroidDists(ic));
end

inFraction = predVectorsWithinBoundaries/size(allPredVectors,2);

clusteringModel.clusterNum = numClusters;
clusteringModel.centroids = C';
clusteringModel.threshDists = meanCentroidDists;

end

