function [poseMat] = getGroundTruth (base_dir)
% Based on KITTI RAW DATA DEVELOPMENT KIT
% 
% Outputs ground truth poses
%
% Input arguments:
% base_dir .... absolute path to sequence base directory (ends with _sync)
% sequence base directory

% load oxts data
oxts = loadOxtsliteData(base_dir);

% transform to poses
pose = convertOxtsToPose(oxts);

poseMat = zeros(4,4,length(pose));

for i = 1:length(pose)
    poseMat(:,:,i) = pose{i};
end