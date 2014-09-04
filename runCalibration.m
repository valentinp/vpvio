addpath('helpers');
addpath('keyframe_imu');
addpath('../MATLAB/utils');
addpath('kitti/devkit');
addpath('kitti');

calibDataDir = '/home/valentin/Desktop/KITTI/2011_09_26/2011_09_26_drive_0119_extract_calibration';
[imuCalibData, ~] = loadImuData(calibDataDir, [0, 10^16]);

accelMean = [mean(imuCalibData.measAccel(1,:)); mean(imuCalibData.measAccel(2,:)); mean(imuCalibData.measAccel(3,:)); ];
accelDemeaned = imuCalibData.measAccel - repmat(accelMean, [1, size(imuCalibData.measAccel, 2)]);
g_w_calib = rotmat_from_quat(imuCalibData.measOrient(:,1))*accelMean;


accel_b_calib = accelMean - rotmat_from_quat(imuCalibData.measOrient(:,1))'*[0; 0; 9.81]


omegaMean = [mean(imuCalibData.measOmega(1,:)); mean(imuCalibData.measOmega(2,:)); mean(imuCalibData.measOmega(3,:));];
omegaDemeaned = imuCalibData.measOmega - repmat(omegaMean, [1, size(imuCalibData.measOmega, 2)]);

omega_b_calib = omegaMean


sigma_a_calib =  [std(accelDemeaned(1,:)); std(accelDemeaned(2,:)); std(accelDemeaned(2,:));]
sigma_g_calib =  [std(omegaDemeaned(1,:)); std(omegaDemeaned(2,:)); std(omegaDemeaned(3,:));]