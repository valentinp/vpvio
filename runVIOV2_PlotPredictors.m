%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('helpers');
addpath('gp');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('kitti/devkit');
addpath('kitti');
addpath('training');
addpath('/home/valentin/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');

if ismac
    addpath('/Users/valentinp/Research/mexopencv');
    addpath('/Users/valentinp/Research/gtsam_toolbox');
else
    addpath('~/mexopencv/');
    addpath('~/Dropbox/Research/Ubuntu/gtsam_toolbox/');
end

import gtsam.*;

%% Parameters
rosBagFileName = '/home/valentin/Desktop/Nexus5Data/2014-09-09-16-56-44.bag';
imuTopic = '/android/imu';
flea3Topic = '/camera/image_raw';

imageSize = [640 480];

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
bag = ros.Bag.load(rosBagFileName);
bag.info()

%% Read all messages 
bagImageData = bag.readAll({flea3Topic});
bagImuData = bag.readAll({imuTopic});
fprintf('Read %i images\n', length(bagImageData));
fprintf('Read %i IMU data points\n', length(bagImuData));

monoImageData = {};
imuData = {};

%% Process images 
monoImageData.rectImages =zeros(imageSize(2), imageSize(1), length(bagImageData));
monoImageData.timestamps = zeros(1, length(bagImageData));
rgbImageData.rectImages =zeros(imageSize(2), imageSize(1), 3, length(bagImageData));
rgbImageData.timestamps = zeros(1, length(bagImageData));

for i=1:length(bagImageData)
    monoImageData.rectImages(:,:,i) = reshape(bagImageData{i}.data(2:3:end), imageSize(1), imageSize(2))';
    monoImageData.timestamps(i) = bagImageData{i}.header.stamp.time;
end

for i=1:length(bagImageData)
    rgbImageData.rectImages(:,:,1,i) = mat2gray(reshape(bagImageData{i}.data(3:3:end), imageSize(1), imageSize(2))');
    rgbImageData.rectImages(:,:,2,i) = mat2gray(reshape(bagImageData{i}.data(2:3:end), imageSize(1), imageSize(2))');
    rgbImageData.rectImages(:,:,3,i) = mat2gray(reshape(bagImageData{i}.data(1:3:end), imageSize(1), imageSize(2))');
    rgbImageData.timestamps(i) = bagImageData{i}.header.stamp.time;
end

%clear bagImageData;

%% Process imu data
imuData.timestamps = [];
imuData.measAccel = [];
imuData.measOrient = [];
imuData.measOmega = [];
i = 1;
initBiasGyroList = [];
initBiasAccelList = [];
for t=1:length(bagImuData)
        if (bagImuData{t}.header.stamp.time > monoImageData.timestamps(1))
         imuData.timestamps(1,i) = bagImuData{t}.header.stamp.time;
         imuData.measAccel(:,i) = bagImuData{t}.linear_acceleration;
         imuData.measOrient(:,i) = [bagImuData{t}.orientation(4); bagImuData{t}.orientation(1:3)];
         imuData.measOmega(:,i) = bagImuData{t}.angular_velocity;
         i = i +1;
        end
end

%% Plot the different predictors
numClusters = 2;
Km = 1; %Controls how far from the mean distance each cluster threshold is
featNum = 500;
[clusteringModel, inFraction] = clusterData(rgbImageData,monoImageData, imuData, numClusters, Km, featNum);


%%
close all;

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
    clusterIds = getClusterIds(predVectors, clusteringModel);
    
    
    imshow(rgbimage);
    hold on;
    plot(keyPointPixels(1, clusterIds == 1), keyPointPixels(2, clusterIds == 1), 'mo' ,'MarkerSize',10);
    plot(keyPointPixels(1, clusterIds == 2), keyPointPixels(2, clusterIds == 2), 'co' ,'MarkerSize',10);
    drawnow();
    pause(0.01);
    clf;
    
end
