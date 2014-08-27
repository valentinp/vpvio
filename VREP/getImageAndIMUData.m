% Copyright 2006-2014 Dr. Marc Andreas Freese. All rights reserved. 
% marc@coppeliarobotics.com
% www.coppeliarobotics.com
% 
% -------------------------------------------------------------------
% This file is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% 
% You are free to use/modify/distribute this file for whatever purpose!
% -------------------------------------------------------------------
%
% This file was automatically created for V-REP release V3.1.0 on January 20th 2014

% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simExtRemoteApiStart(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!
function [VREPrgbImageData, VREPimuData, VREPGroundTruth, timestamps] = getImageAndIMUData()
    frameNum = 50;
    dt = 0.1; %50 ms
    %Initialize variables
    VREPrgbImageData = [];
    VREPGroundTruth = [];
    VREPimuData.measAccel = zeros(3, frameNum);
    VREPimuData.measOrient = zeros(4, frameNum);
    VREPimuData.measOmega = zeros(3,frameNum);
    VREPimuData.initialVelocity = zeros(3,1);
    
	disp('Program started');
	% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
	vrep.simxFinish(-1); % just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',8000,true,true,5000,5);
	
    %Ensure we can connect to the server
   	if (clientID>-1)
			disp('Connected to remote API server');
			[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait);
			if (res==vrep.simx_error_noerror)
				fprintf('Number of objects in the scene: %d\n',length(objs));
            else
				fprintf('Remote API function call returned with error code: %d\n',res);
            end
    else
        disp('Failed connecting to remote API server');
        vrep.simxFinish(clientID); % close the line if still open
        vrep.delete(); % call the destructor!
    end
	   
    %Grab the vision sensor handle
    [err,handle]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_oneshot_wait);
    
    %Use the image to get an image
    [err,res,img]=vrep.simxGetVisionSensorImage2(clientID,handle,1,vrep.simx_opmode_oneshot_wait);
    %IMU Data:
    [err]=vrep.simxGetStringSignal(clientID,'myImuData', vrep.simx_opmode_streaming);

    %Plot the image (note imshow is slower than imagesc)
    close all;
    hImage = imagesc(rand(res(1),res(2),1));
    hold on;
    %hCorners = plot(1, 1, 'g*');
    %colormap(gray)
    i = 1;
    while clientID ~= -1
         %IMU Data   
         [err]=vrep.simxGetStringSignal(clientID,'myImuData', vrep.simx_opmode_buffer);
          if (err==vrep.simx_return_ok)
            [response, IMUstring]=vrep.simxGetStringSignal(clientID,'myImuData',vrep.simx_opmode_streaming);
            IMUdataItem = vrep.simxUnpackFloats(IMUstring)
            VREPimuData.measAccel(:,i) = IMUdataItem(1:3);
            VREPimuData.measOmega(:,i) = IMUdataItem(4:6);
            VREPGroundTruth(:,:,i) = [makeEulerRotation(IMUdataItem(10),IMUdataItem(11),IMUdataItem(12))' -makeEulerRotation(IMUdataItem(10),IMUdataItem(11),IMUdataItem(12))'*IMUdataItem(7:9)'; 0 0 0 1];
            timestamps(i) = IMUdataItem(13);
          end
        %Image data
        [err,res,img]=vrep.simxGetVisionSensorImage2(clientID,handle,0,vrep.simx_opmode_oneshot_wait);
        if res(1) > 0
            VREPrgbImageData(:,:,:,i) = img;
            set(hImage,'CData',img)
            drawnow;
            if i == frameNum
                break;
            end
            i=i+1;
        end
    end
    timestamps = timestamps - timestamps(1);
    vrep.simxFinish(clientID);
	vrep.delete(); % call the destructor!
	disp('Program ended');
    
    function R = makeEulerRotation(alpha, beta, gamma)


        c1 = cos(alpha);
        s1 = sin(alpha);

        c2 = cos(beta);
        s2 = sin(beta);

        c3 = cos(gamma);
        s3 = sin(gamma);

        R = [c1*c3-c2*s1*s3, -c1*s3-c3*c2*s1,  s2*s1;
             c2*c1*s3+c3*s1,  c1*c2*c3-s1*s3, -c1*s2;
             s3*s2         ,  c3*s2         ,    c2 ];

    end
end
