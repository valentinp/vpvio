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

[idx, C,~,D] = kmeans(allPredVectors', numClusters);

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

