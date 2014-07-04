function [T_wIMU, imuData] = genTrajectory(simSetup)
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

%Position parameters (amplitudes and frequencies)
pampl = [0.1;  0.5;   0.5];
pfreq = [0.1;  0.5;  0.5]*pi;

%Orientation parameters (amplitudes and frequencies)
oampl = [-1/20;  1/12;  -1/20]*pi;    
ofreq = [8/50;  3/50;  6/50]*pi;


g_w = [0; -9.81; 0];
forwardSpeed = 0.5;  % m/s

samplingRate = simSetup.imuRate; % Hz
samplingTime = simSetup.simTime;    % seconds


numSteps = samplingRate*samplingTime + 1;

T_wIMU = zeros(4,4,numSteps);
imuData.timestamps = zeros(1, numSteps);
imuData.measAccel = zeros(3, numSteps);
imuData.measOrient = zeros(4, numSteps);
imuData.measOmega = zeros(3, numSteps);
imuData.euler = zeros(3, numSteps);
 
for i = 1:numSteps
    
    % Time
    t = (i-1)/samplingRate;
    
    pang = pfreq*t;
    oang = ofreq*t;    
    
    %----- Orientation -----

    % Assume that initial orientation is identity.
    ewi = angle_normalize(oampl.*sin(oang));
    % ewi = zeros(3,1);
    % Rates of change of Euler angles.
    dewi = oampl.*ofreq.*cos(oang);
    % dewi = zeros(3,1);
    % Angular velocity in IMU frame.
    wi = kinmat_from_rpy(ewi)\dewi;
    %wi = zeros(3,1);
    %----- Position -----
    pwi = forwardSpeed*(t)*[0;0;1] + pampl.*sin(pang);

    % Velocity               
    vw =  forwardSpeed*[0;0;1] + pampl.*pfreq.*cos(pang);
    if i == 1
        imuData.initialVelocity = vw;
    end

    % Acceleration
    aw = -pampl.*pfreq.^2.*sin(pang);
    
    R_wIMU = rotmat_from_rpy(ewi);
    p_IMUw_w = pwi;
    
    
    % Save in struct.
    imuData.timestamps(i) = t - 1e-8;
    T_wIMU(:,:, i) = [R_wIMU  p_IMUw_w; 0, 0, 0, 1];
    imuData.measAccel(:,i)= R_wIMU'*(aw + g_w);
    imuData.measOmega(:,i) = wi;
    imuData.euler(:,i) = ewi;
    
    imuData.measOrient(:,i) = quat_from_rotmat(rotmat_from_rpy(ewi));
    
    
end

end