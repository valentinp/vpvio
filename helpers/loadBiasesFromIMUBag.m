function [ omegaBias,  accelBias, g_w] = loadBiasesFromIMUBag( rosBagFileName, g_mag, useFirstNSeconds)
%LOADBIASESFROMIMUBAG Calculate biases from the first useFirstNSeconds of the given bag file 

%Load the bag file
imuTopic = '/microstrain/imu/data';
bag = ros.Bag.load(rosBagFileName);
bagImuData = bag.readAll({imuTopic});

%Read in all data (TODO: Perhaps not all the data?)
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

%How many samples do we need?
dt = mean(diff(imuData.timestamps));

if useFirstNSeconds == 'ALL'
    sampleNum = length(imuData.timestamps);
else
    sampleNum = round(useFirstNSeconds/dt);
end

%Extract bias and gravity values
linearAccelList = zeros(3,sampleNum);
omegaList = zeros(3,sampleNum);
gList = zeros(3,sampleNum);
for imu_i = 1:sampleNum
       gVec = rotmat_from_quat(imuData.measOrient(:,imu_i))'*[0 0 g_mag]';
       linearAccel = imuData.measAccel(:, imu_i) + gVec;
       linearOmega = imuData.measOmega(:, imu_i);
       linearAccelList(:, imu_i) = linearAccel;
       omegaList(:, imu_i) = linearOmega;
       gList(:, imu_i) = gVec;

end

omegaBias = mean(omegaList,2);
accelBias = mean(linearAccelList,2);
g_w = mean(gList, 2);

end

