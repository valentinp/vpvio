function [T_wIMU, imuData] = genTrajectoryCircle(simSetup)
% GENTRAJECTORY Generate simulated IMU trajectory through the world
%
%   [T_wIMU, imuData] = genTrajectory(type) computes the position,
%   orientation, linear and angular velocity and linear acceleration of a
%   simulated IMU along a smooth sample trajectory.  The function assumes 
%   that the initial velocity of the IMU is zero.
%
%   This function generates a sinusoidal IMU trajectory, with a sinusoidal
%   orientation exitation (on all three axes).  Note the the orientation 
%   portion of the Euler pose vector 'ewi' is ignored (for now).
%
%   The values returned are defined in the world frame, and must be 
%   transformed into the IMU frame to be used as simulated IMU measurements 
%   (except for the angular rates, which are provided in the IMU frame).
%
%   Inputs:
%   -------
%    t             - Current time in seconds.
%    ewi           - 6x1 Euler RPY pose vector for IMU at time t = 0.
%   [setup]        - Struct - path/trajectory settings.
%    setup.pampl   - 3x1 position amplitude vector.
%    setup.pfreq   - 3x1 position frequency vector.
%    setup.oampl   - 3x1 roll, pitch, yaw ampl. vector.
%    setup.ofreq   - 3x1 roll, pitch, yaw freq. vector.
%
%   Outputs:
%   --------


g_w = [0; 0; -9.81];
forwardSpeed = 2;  % m/s
circleRadius = 10; %m
period = 2*pi*circleRadius/forwardSpeed;

samplingRate = simSetup.imuRate; % Hz
samplingTime = simSetup.simTime;    % seconds


numSteps = samplingRate*samplingTime + 1;

T_wIMU = zeros(4,4,numSteps);
imuData.timestamps = zeros(1, numSteps);
imuData.measAccel = zeros(3, numSteps);
imuData.measOrient = zeros(4, numSteps);
imuData.measOmega = zeros(3, numSteps);
imuData.euler = zeros(3, numSteps);

b_a = 0;
b_w = 0;

for i = 1:numSteps
    
    n_a = simSetup.accelNoiseStd*randn(3,1);
    n_w = simSetup.gyroNoiseStd*randn(3,1);
    b_a = b_a + (1/samplingRate)*simSetup.gyroBiasStd*randn(3,1);
    b_w = b_w + (1/samplingRate)*simSetup.accelBiasStd*randn(3,1);
    
    
    % Time
    t = (i-1)/samplingRate;
    
    ang = t*(2*pi/period);
    
    %----- Orientation -----

    % Assume that initial orientation is identity.
    ewi = angle_normalize([-pi/2;0;ang]);
    % ewi = zeros(3,1);
    % Rates of change of Euler angles.
    dewi = [0;0;2*pi/period];
    % dewi = zeros(3,1);
    % Angular velocity in IMU frame.
    wi = kinmat_from_rpy(ewi)\dewi;
    %wi = zeros(3,1);
    %----- Position -----
    pwi = circleRadius*[cos(ang); sin(ang); 0] + [0;0;1];

    % Velocity               
    vw = circleRadius*(2*pi/period)*[-sin(ang); cos(ang); 0];
    if i == 1
        imuData.initialVelocity = vw;
        imuData.initialPosition = pwi;
    end

    % Acceleration
    aw = circleRadius*(2*pi/period)^2*[-cos(ang); -sin(ang); 0];
    
    R_wIMU = rotmat_from_rpy(ewi);
    p_IMUw_w = pwi;
    
    
    % Save in struct.
    if i == 1
        imuData.timestamps(i) = 1e-8;
    else
        imuData.timestamps(i) = t - 1e-8;
    end
    T_wIMU(:,:, i) = [R_wIMU  p_IMUw_w; 0, 0, 0, 1];
    imuData.measAccel(:,i)= R_wIMU'*(aw - g_w + n_a + b_a);
    imuData.measOmega(:,i) = wi + n_w + b_w;
    imuData.euler(:,i) = ewi;
    
    imuData.measOrient(:,i) = quat_from_rotmat(rotmat_from_rpy(ewi));
    
    
end

end