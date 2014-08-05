%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('../helpers');
addpath('../keyframe_imu');
addpath('../../MATLAB/utils');
addpath('../kitti/devkit');
addpath('../kitti');
addpath('~/Dropbox/Research/Ubuntu/opengv/matlab');%
%addpath('~/mexopencv/');


%% Where is the data?
%Karslrugh city centre
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_29/2011_09_29_drive_0071_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_29';

%Open street
dataBaseDir =  '/Users/valentinp/Research/Datasets/Kitti/2011_09_29/2011_09_29_drive_0071_sync';
dataCalibDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_29';
 %Foresty road
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0028_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

 %Cityish
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0059_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%Trail through forest
% dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0087_sync';
% dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%Turn
%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_30/2011_09_30_drive_0027_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_30';

%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0002_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

%dataBaseDir =  '/home/valentin/Desktop/KITTI/2011_09_30/2011_09_30_drive_0027_sync';
%dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_30';

%% Options
%Pipeline
pipelineOptions.featureDetector = 'SURF';
pipelineOptions.featureCount = 5000;
pipelineOptions.descriptorExtractor = 'SURF';
pipelineOptions.descriptorMatcher = 'BruteForce';
pipelineOptions.minMatchDistance = 0.5;





%% Get prediction vectors
%Define the frames
frameRange = 1:30;

%Image data
monoImageData = loadImageData([dataBaseDir '/image_00'], frameRange);
rgbImageData = loadImageDataRGB([dataBaseDir '/image_02'], frameRange);

allPredVectors = [];

for i = 1:size(monoImageData.rectImages, 3)
    image = uint8(monoImageData.rectImages(:,:,i));
    rgbimage = rgbImageData.rectImages(:,:,:,i);
    
    points = detectSURFFeatures(image);
    locations = round(points.Location(:,:)');
    
    %Computes Prediction Space points based on the image and keypoint position
    predVectors = computePredVectors(locations, rgbimage);
    allPredVectors = [allPredVectors predVectors];
    
    
end

%% Cluster the features
numClusters = 10;
[idx, C, sumd, D] = kmeans(allPredVectors', numClusters);

%Determine the mean distance of points within a cluster to the cluster's centroid
%This sets boundaries

%Controls how far from the mean distance each cluster threshold is
Km = 1.5;

meanCentroidDists = zeros(numClusters, 1);
for ic = 1:numClusters
    meanCentroidDists(ic) = Km*mean(D(idx == ic, ic));
end

%Possibly apply PCA?
%[COEFF,SCORE] = princomp(allPredVectors');

    
% Step through each image and remove one cluster at a time

for i = 1:size(monoImageData.rectImages, 3)
    image = uint8(monoImageData.rectImages(:,:,i));
    rgbimage = rgbImageData.rectImages(:,:,:,i);
    
    points = detectSURFFeatures(image);
    locations = round(points.Location(:,:)');
    
    %Computes Prediction Space points based on the image and keypoint position
    predVectors = computePredVectors(locations, rgbimage);
    
    % Remove each cluster
    
    totalRemoved = 0;
    for ic = 1:numClusters
        %printf('%d keypoints before cluster removal. ', size(locations,2));
            
        locationsSansCluster = removeCluster(locations, predVectors, C(ic, :)', meanCentroidDists(ic));
        %printf('%d keypoints after cluster removal. ', size(locationsSansCluster,2));
         totalRemoved = totalRemoved + (size(locations,2) - size(locationsSansCluster,2));
        
    end
     printf('Clusters correspond to %d/%d keypoints', totalRemoved, size(locations,2));
end


