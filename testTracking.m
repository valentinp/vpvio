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

dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0036_sync';
dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

frameRange = 1:200;

%Images
rosBagFileName = '/home/valentin/Desktop/Crucifix/2014-09-19-19-14-47.bag';
imageSize = [1280 960];
flea3Topic = '/flea3/camera/image_rect';
bag = ros.Bag.load(rosBagFileName);
bag.info()
bagImageData = bag.readAll({flea3Topic});

%%
currImage = reshape(bagImageData{1}.data, imageSize(1), imageSize(2))';
keyPoints = detectMinEigenFeatures(mat2gray(currImage));
KLNewkeyPointPixels = keyPoints.Location(:,:)';

for img_i = 2:length(bagImageData)
    oldImage = currImage;
    oldkeyPointPixels = KLNewkeyPointPixels;
    
    currImage = reshape(bagImageData{img_i}.data, imageSize(1), imageSize(2))';
    keyPoints = detectHarrisFeatures(mat2gray(currImage));
    keyPointPixels = keyPoints.Location(:,:)';
    
    KLOldKeyPoints = num2cell(double(oldkeyPointPixels'), 2)';
    [KLNewKeyPoints, status, ~] = cv.calcOpticalFlowPyrLK(uint8(oldImage), uint8(currImage), KLOldKeyPoints);
    KLOldkeyPointPixels = cell2mat(KLOldKeyPoints(:))';
    KLNewkeyPointPixels = cell2mat(KLNewKeyPoints(:))';
    showMatchedFeatures(oldImage,currImage, KLOldkeyPointPixels', KLNewkeyPointPixels');
    drawnow;
    pause(0.001);
          
end