function [ omegaBias,  accelBias] = loadBiasesFromIMUBag( rosBagFileName )
%LOADBIASESFROMIMUBAG Summary of this function goes here
%   Detailed explanation goes here

imuTopic = '/microstrain/imu/data';

bag = ros.Bag.load(rosBagFileName);
bagImuData = bag.readAll({imuTopic});
imuData = {};
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

g_mag = 9.805;
% First calculate the bias by subtracting the gravity vector (which we know is upwards)
% Use the first 10 seconds of data
linearAccelList = zeros(3,length(imuData.timestamps));
omegaList = zeros(3,length(imuData.timestamps));
for imu_i = 1:length(imuData.timestamps)
       linearAccel = imuData.measAccel(:, imu_i) + rotmat_from_quat(imuData.measOrient(:,imu_i))'*[0 0 g_mag]';
       linearOmega = imuData.measOmega(:, imu_i);
       linearAccelList(:, imu_i) = linearAccel;
       omegaList(:, imu_i) = linearOmega;
end

omegaBias = mean(omegaList,2);
accelBias = mean(linearAccelList,2);


end

