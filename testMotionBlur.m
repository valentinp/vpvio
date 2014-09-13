addpath('helpers');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('kitti/devkit');
addpath('kitti');
%Train
dataBaseDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0036_sync';
dataCalibDir = '/home/valentin/Desktop/KITTI/2011_09_26';

% Get ground truth and import data
frameRange = 1:200;

%Image data
monoImageData = loadImageData([dataBaseDir '/image_00'], frameRange);

%IMU data
[imuData, imuFrames] = loadImuData(dataBaseDir, monoImageData.timestamps);
%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir, imuFrames);

%%


for image_i = 3:size(monoImageData.rectImages,3)
    if norm(imuData.measOmega(:, image_i)) > 0.2
    printf('Blurring image %d', image_i)    
    blurredImage = 0.5*mat2gray(monoImageData.rectImages(:,:,image_i)) + 0.25*mat2gray(monoImageData.rectImages(:,:,image_i-1)) + 0.25*mat2gray(monoImageData.rectImages(:,:,image_i-2));
    imshow(blurredImage);
    else
    imshow(mat2gray(monoImageData.rectImages(:,:,image_i)));
    end
    drawnow();
    pause(0.03);
end
