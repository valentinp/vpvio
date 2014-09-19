%% Clean up
clc;
clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;
addpath('helpers');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('../MATLAB/kinematics_toolbox/screws');

addpath('kitti/devkit');
addpath('kitti');
if ismac
addpath('~/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-mac64/');
else
    addpath('~/Dropbox/UTIAS - MASc/Code/MATLAB/matlab_rosbag-0.4-linux64/');

end
%%
rosBagFileName = '/home/valentin/Desktop/Crucifix/2014-09-16-20-12-15.bag';
imuTopic = '/microstrain/imu/data';
bag = ros.Bag.load(rosBagFileName);
bag.info()
bagImuData = bag.readAll({imuTopic});


imuData.timestamps = [];
imuData.measAccel = [];
imuData.measOrient = [];
imuData.measOmega = [];

for t=1:length(bagImuData)
         i = t;   
         imuData.timestamps(1,i) = bagImuData{t}.header.stamp.time;
         imuData.measOrient(:,i) = [bagImuData{t}.orientation(4); bagImuData{t}.orientation(1:3)];
         imuData.measAccel(:,i) = bagImuData{t}.linear_acceleration;
         imuData.measOmega(:,i) = bagImuData{t}.angular_velocity;
end

% csvFile = '~/Desktop/500Hzmotion.csv';
% imuDataCSV = csvread(csvFile);
% 
% imuData.timestamps = [];
% imuData.measAccel = [];
% imuData.measOrient = [];
% imuData.measOmega = [];
% imuData.measPRH = [];
% 
% for i=1:size(imuDataCSV,1)
%          imuData.timestamps(i) = imuDataCSV(i,1);
%          imuData.measAccel(:,i) = 9.80665*imuDataCSV(i,5:7)';
%          imuData.measOmega(:,i) = imuDataCSV(i,8:10)';
%          imuData.measPRH(:,i) = imuDataCSV(i,2:4)/180*pi;
% end
%==============
% jkFile = '~/Desktop/timing-1.txt';
% imuDataCSV = importdata(jkFile, ' ');
% imuData.timestamps = [];
% imuData.measAccel = [];
% imuData.measOrient = [];
% imuData.measOmega = [];
% imuData.measPRH = [];
% 
% for i=1:size(imuDataCSV,1)
%          imuData.timestamps(i) = imuDataCSV(i,1);
%          imuData.measAccel(:,i) = 9.80665*imuDataCSV(i,3:5)';
%          imuData.measOmega(:,i) = imuDataCSV(i,6:8)';
% end
%================
% csvFile = '~/Desktop/deltaAngledetlaV.csv';
% imuDataCSV = csvread(csvFile);
% 
% imuData.timestamps = [];
% imuData.measAccel = [];
% imuData.measOrient = [];
% 
% for i=1:size(imuDataCSV,1)
%          imuData.timestamps(i) = imuDataCSV(i,1);
%          imuData.measAccel(:,i) = 9.80665*imuDataCSV(i,5:7)'/(1/500);
%          imuData.measOmega(:,i) = imuDataCSV(i,2:4)'/(1/500);
% end

%%
close all
dt = 1/500;
biasSec = 3;
g_mag = 9.805;
% First calculate the bias by subtracting the gravity vector (which we know is upwards)
% Use the first 10 seconds of data
linearAccelList = zeros(3,biasSec/dt);
omegaList = zeros(3,biasSec/dt);

for imu_i = 1:(biasSec/dt)
       
%        linearAccel = imuData.measAccel(:, imu_i) + testConvertPRHtoM(imuData.measPRH(:, imu_i))*[0 0 g_mag]';
       
       linearAccel = imuData.measAccel(:, imu_i) + rotmat_from_quat(imuData.measOrient(:,imu_i))'*[0 0 g_mag]';
       linearOmega = imuData.measOmega(:, imu_i);
       
       linearAccelList(:, imu_i) = linearAccel;
       omegaList(:, imu_i) = linearOmega;
end

omegaBias = mean(omegaList,2)
accelBias = mean(linearAccelList,2)
%%
figure
subplot(1,3,1)
xAccel = imuData.measAccel(1,:)- accelBias(1);
hist(xAccel, 50)
title(sprintf('X \n Mean: %.5f \n Median: %.5f \n Min: %.5f \n Max: %.5f \n STD: %.5f', mean(xAccel), median(xAccel), min(xAccel),max(xAccel),std(xAccel)))

subplot(1,3,2)
yAccel = imuData.measAccel(2,:)- accelBias(2);
hist(yAccel, 50)
title(sprintf('Y \n Mean: %.5f \n Median: %.5f \n Min: %.5f \n Max: %.5f \n STD: %.5f', mean(yAccel), median(yAccel), min(yAccel),max(yAccel),std(yAccel)))

subplot(1,3,3)
zAccel = imuData.measAccel(3,:)- accelBias(3) + g_mag;
hist(zAccel, 50)
title(sprintf('Z \n Mean: %.5f \n Median: %.5f \n Min: %.5f \n Max: %.5f \n STD: %.5f', mean(zAccel), median(zAccel), min(zAccel),max(zAccel),std(zAccel)))

%%
g_w = [0;0;g_mag];
noiseParams = [];

xInit.p = zeros(3,1);
xInit.v = zeros(3,1);
xInit.b_g = omegaBias;
xInit.b_a = accelBias;
xInit.q = [1; zeros(3,1)];

xState = xInit;
rpy_list = zeros(3, length(imuData.timestamps));
pos_list = zeros(3, length(imuData.timestamps));

for imu_i = 1:length(imuData.timestamps)
     if imu_i > 1
         dt = imuData.timestamps(imu_i) - imuData.timestamps(imu_i-1);
     end
     imuAccel = imuData.measAccel(:, imu_i);
     imuOmega = imuData.measOmega(:, imu_i);
     
     %Use the magnetometer
     imuAccel = imuData.measAccel(:, imu_i) + rotmat_from_quat(imuData.measOrient(:,imu_i))'*[0 0 g_mag]';
     g_w = zeros(3,1);
     
     %Integrate
     [xState] = integrateIMU(xState, imuAccel, imuOmega, dt, noiseParams, g_w);
     pos_list(:, imu_i) = xState.p;
     rpy_list(:,imu_i) = rpy_from_rotmat(rotmat_from_quat(xState.q))*180/pi;
end



%Plot the final result
figure; 
subplot(2,3,1);
plot(pos_list(1,:))
title('X');
subplot(2,3,2);
plot(pos_list(2,:))
title('Y');
subplot(2,3,3);
plot(pos_list(3,:))
title('Z');
subplot(2,3,4);
plot(rpy_list(1,:))
title('Roll');
subplot(2,3,5);
plot(rpy_list(2,:))
title('Pitch');
subplot(2,3,6);
plot(rpy_list(3,:))
title('Yaw');
%%
figure;
plot3(pos_list(1,:),pos_list(2,:),pos_list(3,:))
