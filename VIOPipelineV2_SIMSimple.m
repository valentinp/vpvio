function [T_wcam_estimated,T_wimu_estimated,T_wimu_gtsam, keyFrames] = VIOPipelineV2_SIMSimple(K, T_camimu, imageMeasurements, imuData, pipelineOptions, noiseParams, xInit, g_w, T_wImu_GT)
%VIOPIPELINE Run the Visual Inertial Odometry Pipeline
% K: camera intrinsics
% T_camimu: transformation from the imu to the camera frame
% imuData: struct with IMU data:
%           imuData.timestamps: 1xN 
%           imuData.measAccel: 3xN
%           imuData.measOmega: 3xN
%           imuData.measOrient: 4xN (quaternion q_sw, with scalar in the
%           1st position. The world frame is defined as the N-E-Down ref.
%           frame.
% imageMeasurements:
%           array of imageMeasurement structs:
%           imageMeasurements(i).timestamp
%           imageMeasurements(i).pixelMeasurements (2xN)
%           imageMeasurements(i).landmarkIds (Nx1)

% params:
%           params.INIT_DISPARITY_THRESHOLD
%           params.KF_DISPARITY_THRESHOLD
%           params.MIN_FEATURE_MATCHES

 import gtsam.*;

%===GTSAM INITIALIATION====%
currentPoseGlobal = Pose3(Rot3(rotmat_from_quat(xInit.q)), Point3(xInit.p)); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector(xInit.v); 
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_between_b = [ noiseParams.sigma_ba ; noiseParams.sigma_bg ];
w_coriolis = [0;0;0];



% Solver object
isamParams = ISAM2Params;
isamParams.setRelinearizeSkip(1);
gnParams = ISAM2GaussNewtonParams;
%gnParams.setWildfireThreshold(1000);
isamParams.setOptimizationParams(gnParams);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;
%==========================%


referencePose = {};

%Key frame poses correspond to the first and second poses from which 
%point clouds are triangulated (these must have sufficient disparity)
keyFrames = [];
keyFrame_i = 1;
initiliazationComplete = false;




% Main loop
% ==========================================================
% Sort all measurements by their timestamps, process measurements as if in
% real-time

%Extract image timestamps
imageTimestamps = zeros(1, length(imageMeasurements));
for i = 1:length(imageMeasurements)
    imageTimestamps(i) = imageMeasurements(i).timestamp;
end

%All measurements are assigned a unique measurement ID based on their
%timestamp
numImageMeasurements = length(imageTimestamps);
numImuMeasurements = length(imuData.timestamps);
numMeasurements = numImuMeasurements + numImageMeasurements;

allTimestamps = [imageTimestamps imuData.timestamps];
[~,measIdsTimeSorted] = sort(allTimestamps); %Sort timestamps in ascending order
 
camMeasId = 0;
imuMeasId = 0;



%Initialize the state
xPrev = xInit;
xDeadReckon = xInit;
%Initialize the history



%Initialize the history
R_wimu = rotmat_from_quat(xPrev.q);
R_imuw = R_wimu';
p_imuw_w = xPrev.p;
T_wimu_estimated = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
T_wcam_estimated = T_wimu_estimated*inv(T_camimu);
T_wimu_gtsam = [];


iter = 1;

%Keep track of landmarks
initializedLandmarkIds = [];

initialObservations.pixels = [];
initialObservations.poseKeys = [];
initialObservations.ids =  [];

pastObservations.pixels = [];
pastObservations.poseKeys = [];
pastObservations.ids =  [];


for measId = measIdsTimeSorted
    % Which type of measurement is this?
    if measId > numImageMeasurements
        measType = 'IMU';
        imuMeasId = measId - numImageMeasurements;
    else 
        measType = 'Cam';
        camMeasId = measId;
        %continue;
    end
    
    
    % IMU Measurement
    % ==========================================================
    if strcmp(measType, 'IMU')

        
        
        %Calculate dt
        if imuMeasId ~= numImuMeasurements
            dt = imuData.timestamps(imuMeasId+1) - imuData.timestamps(imuMeasId);
        end
        
        
        %Extract the measurements
        imuAccel = imuData.measAccel(:, imuMeasId);
        imuOmega = imuData.measOmega(:, imuMeasId);         
          

        %Predict the next state
        [xPrev] = integrateIMU(xPrev, imuAccel, imuOmega, dt, noiseParams, g_w);
        [xDeadReckon] = integrateIMU(xDeadReckon, imuAccel, imuOmega, dt, noiseParams, g_w);


        %=======GTSAM=========
        %Integrate each measurement
        currentSummarizedMeasurement.integrateMeasurement(imuAccel, imuOmega, dt);
        %=====================
        
        %Formulate matrices 
        R_wimu = rotmat_from_quat(xDeadReckon.q);
        R_imuw = R_wimu';
        p_imuw_w = xDeadReckon.p;
        
        
        %Keep track of the state
        T_wimu_estimated(:,:, end+1) = inv([R_imuw -R_imuw*p_imuw_w; 0 0 0 1]);
        
   
    % Camera Measurement 
    % ==========================================================
    elseif strcmp(measType, 'Cam')
     
        
        %The last IMU state based on integration (relative to the world)
        T_wimu_int = [rotmat_from_quat(xPrev.q) xPrev.p; 0 0 0 1];

        
        %If it's the first image, set the current pose to the initial
        %keyFramePose
        if camMeasId == 1
           referencePose.T_wimu_int = T_wimu_int;
           referencePose.T_wimu_opt = T_wimu_int;
           referencePose.T_wcam_opt = T_wimu_int*inv(T_camimu);
           referencePose.imuMeasId = 1;
          
            % =========== GTSAM ============
            % Initialization
            currentPoseKey = symbol('x',1);
            currentVelKey =  symbol('v',1);
            currentBiasKey = symbol('b',1);

            %Initialize the state
            newValues.insert(currentPoseKey, currentPoseGlobal);
             newValues.insert(currentVelKey, currentVelocityGlobal);
            newValues.insert(currentBiasKey, currentBias);
            
            %Add constraints
            %newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
            newFactors.add(NonlinearEqualityPose3(currentPoseKey, currentPoseGlobal));
            newFactors.add(NonlinearEqualityLieVector(currentVelKey, currentVelocityGlobal));
             newFactors.add(NonlinearEqualityConstantBias(currentBiasKey, currentBias));
            
            %Prepare for IMU Integration
            currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
                      currentBias, diag(noiseParams.sigma_a.^2), ...
                      diag(noiseParams.sigma_g.^2), 1e-2 * eye(3));
                
            %Note: We cannot add landmark observations just yet because we
            %cannot be sure that all landmarks will be observed from the
            %next pose (if they are not, the system is underconstrained and  ill-posed)
           
            % ==============================
           
        else
                 
              T_rimu = inv(referencePose.T_wimu_opt)*T_wimu_int;
              
              T_rcam = T_camimu*T_rimu*inv(T_camimu);
              R_rcam = T_rcam(1:3,1:3);
              p_camr_r = homo2cart(T_rcam*[0 0 0 1]');

             

           


                disp(['Creating new keyframe: ' num2str(keyFrame_i)]);   

                     %=========== GTSAM ===========
        
        % At each non=IMU measurement we initialize a new node in the graph
          currentPoseKey = symbol('x',keyFrame_i+1);
          currentVelKey =  symbol('v',keyFrame_i+1);
          currentBiasKey = symbol('b',keyFrame_i+1);
  
             %Important, we keep track of the optimized state and 'compose'
      %odometry onto it!
      currPose = Pose3(referencePose.T_wimu_opt*T_rimu);
   
             % Summarize IMU data between the previous GPS measurement and now
               newFactors.add(ImuFactor( ...
       currentPoseKey-1, currentVelKey-1, ...
       currentPoseKey, currentVelKey, ...
      currentBiasKey, currentSummarizedMeasurement, g_w, w_coriolis));
  
  %Prepare for IMU Integration
            currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
                      currentBias, diag(noiseParams.sigma_a.^2), ...
                      diag(noiseParams.sigma_g.^2), 1e-2 * eye(3));
             
             
       newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), noiseModel.Diagonal.Sigmas(sqrt(40) * sigma_between_b)));

    newValues.insert(currentPoseKey, currPose);
     newValues.insert(currentVelKey, currentVelocityGlobal);
     newValues.insert(currentBiasKey, currentBias);
    
        %=============================
        
        
  
                    
        
        
                 if camMeasId == 2
                    newFactors.add(NonlinearEqualityPose3(currentPoseKey, currPose));
                 end
                    

                    
                    
                        
                
        
                       
                        isam.update(newFactors, newValues);
                        isamCurrentEstimate = isam.calculateEstimate();

                        
                        
                        %Reset the new values
                        newFactors = NonlinearFactorGraph;
                         newValues = Values;

                         %isam.getDelta()
                    %isam.getLinearizationPoint()
                    isamCurrentEstimate.at(currentVelKey)
                    xPrev.v

                   
                               currentVelocityGlobal = isamCurrentEstimate.at(currentVelKey);
                currentBias = isamCurrentEstimate.at(currentBiasKey);
                currentPoseGlobal = isamCurrentEstimate.at(currentPoseKey);
                
                currentPoseTemp = currentPoseGlobal.matrix;
                xPrev.p = currentPoseTemp(1:3,4); 
                 xPrev.q = quat_from_rotmat(currentPoseTemp(1:3, 1:3));
                 xPrev.v =  currentVelocityGlobal.vector; %Note velocity has to be in the reference frame!
                xPrev.b_a = currentBias.accelerometer;
                xPrev.b_g = currentBias.gyroscope;

                
                %Plot the results
                p_wimu_w = currentPoseGlobal.translation.vector;
                p_wimu_w_int = T_wimu_int(1:3,4);
                plot(p_wimu_w(1), p_wimu_w(2), 'g*');
                plot(p_wimu_w_int(1), p_wimu_w_int(2), 'r*');
                set (gcf(), 'outerposition', [25 800, 560, 470])
                hold on;
                drawnow;
                pause(0.01);
                
                 


               %Save keyframe
               %Each keyframe requires:
               % 1. Absolute rotation and translation information (i.e. pose)
               % 2. Triangulated 3D points and associated descriptor vectors

               keyFrames(keyFrame_i).imuMeasId = imuMeasId+1;
               keyFrames(keyFrame_i).T_wimu_opt = currentPoseGlobal.matrix;
               keyFrames(keyFrame_i).T_wimu_int = T_wimu_int;
               keyFrames(keyFrame_i).T_wcam_opt = currentPoseGlobal.matrix*inv(T_camimu);
              

               %Update the reference pose
               referencePose = {};
               referencePose = keyFrames(keyFrame_i);

               
                keyFrame_i = keyFrame_i + 1;
           
           
        end % if camMeasId == 1

    end % strcmp(measType...)
    
    iter = iter + 1;
end % for measId = ...
%Output the final estimate
for kf_i = 1:(keyFrame_i-1)
    T_wimu_gtsam(:,:, kf_i) = isamCurrentEstimate.at(symbol('x', kf_i+1)).matrix;
end
end

