% x = -50:0.01:50;
% 
% y = 100./(1+x.^2).^2;
% plot(x,y)
% 
% %%
% ids = getClusterIds(allPredVectors, clusteringModel);
% y = zeros(1,10);
% for id = 1:10
%     y(id) = sum(ids == id);
% end
% figure
% bar(1:10, y)

%% Test KLT
%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('helpers');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('kitti/devkit');
addpath('kitti');
addpath('vrep');
if ismac
    addpath('/Users/valentinp/Research/opengv/matlab');
    addpath('/Users/valentinp/Research/mexopencv');
else
    addpath('~/Dropbox/Research/Ubuntu/opengv/matlab');
    addpath('~/mexopencv/');
end

%%
%% Get ground truth and import data
% Synthetic data from VREP
[T_wIMU_GT, monoImageData, rgbImageData, imuData] = processVREP();

%% Options
%Pipeline
pipelineOptions.featureDetector = 'SURF';
pipelineOptions.featureCount = 5000;
pipelineOptions.descriptorExtractor = 'SURF';
pipelineOptions.descriptorMatcher = 'BruteForce';
pipelineOptions.minMatchDistance = 0.5;
%% Get prediction vectors
%Define the frames
%import cv.*;


%Image data
opencvDetector = cv.FeatureDetector(pipelineOptions.featureDetector);
opencvExtractor = cv.DescriptorExtractor(pipelineOptions.descriptorExtractor);
allPredVectors = [];

pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

for i = 1:(size(monoImageData.rectImages, 3) - 1)
    image = uint8(monoImageData.rectImages(:,:,i));
    nextImage = uint8(monoImageData.rectImages(:,:,i+1));
    
        
    surfPoints = opencvDetector.detect(image);
    [~,sortedKeyPointIds] = sort([surfPoints.response]);
    surfPoints = surfPoints(sortedKeyPointIds(1:min(pipelineOptions.featureCount,length(surfPoints))));
    pixel_locations = reshape([surfPoints.pt], [2 length(surfPoints)]);

    %currPts = cv.goodFeaturesToTrack(image);
    currPts = num2cell(pixel_locations, 1);
    nextPts = cv.calcOpticalFlowPyrLK(image, nextImage, currPts);
    
    currPtsArray = pixel_locations';
    nextPtsArray = cell2mat(nextPts(:));
    
    % Remove any points that have negative coordinates
    negCoordIdx = nextPtsArray(:,1) < 0 | nextPtsArray(:,2) < 0;
    currPtsArray(negCoordIdx, :) = []; 
    nextPtsArray(negCoordIdx, :) = []; 
   
    
    %Use MATLAB for SURF feature extraction
    nextSurfPoints = SURFPoints(nextPtsArray);
    surfFeatures = extractFeatures(nextImage, nextSurfPoints);
    surfFeaturesCV = opencvExtractor.compute(nextImage, surfPoints);
    
          showMatchedFeatures(image,nextImage, currPtsArray, nextPtsArray);
          drawnow;
         pause(0.01);
            
end
