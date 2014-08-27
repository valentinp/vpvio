function [imuData] = loadImuData(imuDataFolder, frameRange)
% loadImageData Read captured image data into memory. Tries to load
% individual images or a saved mat file if one exists.
    
    frameNum = length(frameRange);

    %Read IMU data
    oxtsData = loadOxtsliteData(imuDataFolder,frameRange);
    % load IMU Data
    v_index = [9:11]; % FLU frame
    a_index = 12:14; % 12:14 body frame, 15:17 FLU frame
    omega_index = 18:20; % 18:20 body frame, 21:23 FLU frame

       
    dateStrings = loadTimestamps([imuDataFolder '/oxts']);
    dateStrings = dateStrings(frameRange);
    timestamps = zeros(1, length(dateStrings));
    for i = 1:length(dateStrings)
        timestamps(i) =  datenum_to_unixtime(datenum(dateStrings(i))) + 0.01;
    end
    

    imuData.timestamps = zeros(1, frameNum);
    imuData.measAccel = zeros(3, frameNum);
    imuData.measOrient = zeros(4, frameNum);
    imuData.measOmega = zeros(3,frameNum);
    imuData.initialVelocity = zeros(3,1);
   
    % for all oxts packets do
    for i=1:length(oxtsData)

      % if there is no data 
      if isempty(oxtsData{i})
        continue;
      end
      
      if i == 1
          imuData.initialVelocity = getRnb(oxtsData{1})'*[ oxtsData{1}(8); oxtsData{1}(7); 0; ];
      end
      
     imuData.timestamps(1,i) = timestamps(i);
     imuData.measAccel(:,i) =  oxtsData{i}(a_index)';
     imuData.measOmega(:,i) = oxtsData{i}(omega_index);

      
      rx = oxtsData{i}(4); % roll
      ry = oxtsData{i}(5); % pitch
      rz = oxtsData{i}(6); % heading 
      Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % base => nav  (level oxts => rotated oxts)
      Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % base => nav  (level oxts => rotated oxts)
      Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)
      R  = Rz*Ry*Rx;

      imuData.measOrient(:,i) = quat_from_rotmat(R);
     
    end
    
    function dn = datenum_to_unixtime( date_num )
      dn =  (date_num - 719529)*86400;         %# 719529 == datenum(1970,1,1)
    end
    
end