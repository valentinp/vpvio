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


%% Where is the data?
%Open street
%OSX
%dataBaseDir =  '/Users/valentinp/Research/Datasets/Kitti/2011_09_26/2011_09_26_drive_0009_sync';
%dataCalibDir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_26';

%Ubuntu
dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0002_sync';
dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';



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
frameRange = 1:50;

%Image data
monoImageData = loadImageData([dataBaseDir '/image_00'], frameRange);
rgbImageData = loadImageDataRGB([dataBaseDir '/image_02'], frameRange);
opencvDetector = cv.FeatureDetector(pipelineOptions.featureDetector);
opencvExtractor = cv.DescriptorExtractor(pipelineOptions.descriptorExtractor);


allPredVectors = [];

pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
initialize(pointTracker, points, videoFrame);


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

%%
% Create a cascade detector object.
faceDetector = vision.CascadeObjectDetector();

% Read a video frame and run the face detector.
videoFileReader = vision.VideoFileReader('tilted_face.avi');
videoFrame      = step(videoFileReader);
bbox            = step(faceDetector, videoFrame);

% Convert the first box to a polygon.
% This is needed to be able to visualize the rotation of the object.
x = bbox(1, 1); y = bbox(1, 2); w = bbox(1, 3); h = bbox(1, 4);
bboxPolygon = [x, y, x+w, y, x+w, y+h, x, y+h];

% Draw the returned bounding box around the detected face.
videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon);

figure; imshow(videoFrame); title('Detected face');

% Detect feature points in the face region.
points = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', bbox);

% Display the detected points.
figure, imshow(videoFrame), hold on, title('Detected features');
plot(points);

% Create a point tracker and enable the bidirectional error constraint to
% make it more robust in the presence of noise and clutter.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Initialize the tracker with the initial point locations and the initial
% video frame.
points = points.Location;
initialize(pointTracker, points, videoFrame);

videoPlayer  = vision.VideoPlayer('Position',...
    [100 100 [size(videoFrame, 2), size(videoFrame, 1)]+30]);

% Make a copy of the points to be used for computing the geometric
% transformation between the points in the previous and the current frames
oldPoints = points;

while ~isDone(videoFileReader)
    % get the next frame
    videoFrame = step(videoFileReader);

    % Track the points. Note that some points may be lost.
    [points, isFound] = step(pointTracker, videoFrame);
    visiblePoints = points(isFound, :);
    oldInliers = oldPoints(isFound, :);

    if size(visiblePoints, 1) >= 2 % need at least 2 points

        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
            oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

        % Apply the transformation to the bounding box
        [bboxPolygon(1:2:end), bboxPolygon(2:2:end)] ...
            = transformPointsForward(xform, bboxPolygon(1:2:end), bboxPolygon(2:2:end));

        % Insert a bounding box around the object being tracked
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon);

        % Display tracked points
        videoFrame = insertMarker(videoFrame, visiblePoints, '+', ...
            'Color', 'white');

        % Reset the points
        oldPoints = visiblePoints;
        setPoints(pointTracker, oldPoints);
    end

    % Display the annotated video frame using the video player object
    step(videoPlayer, videoFrame);
end

% Clean up
release(videoFileReader);
release(videoPlayer);
release(pointTracker);
