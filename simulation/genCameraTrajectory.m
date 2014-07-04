function T_cw = genCameraTrajectory(type, numSteps)
%GENCAMERATRAJECTORY Generate a camera trajectory (successive
%transformations)
addpath([getenv('HOME') '/Dropbox/UTIAS - MASc/Code/MATLAB/utils']);

    if strcmp(type, 'shuffleRight')
       T_step = [eye(3) [0.5 0 0]'; 0 0 0 1];
       T_cw = zeros(4,4,numSteps);
       T_cw(:,:,1) = [eye(3) zeros(3,1); 0 0 0 1];
       for i = 2:numSteps
           T_cw(:,:, i) = T_step*T_cw(:,:, i-1);
       end
    elseif strcmp(type, 'squigle')
       T_step_l = [rotyd(5) [1/sqrt(2) 1/sqrt(2) 0]'; 0 0 0 1];
       T_step_r = [rotyd(5) [1/sqrt(2) 1/sqrt(2) 0]'; 0 0 0 1];

       T_cw = zeros(4,4,numSteps);
       T_cw(:,:,1) = [eye(3) zeros(3,1); 0 0 0 1];
       for i = 2:floor(numSteps/2)
           T_cw(:,:, i) = T_step_l*T_cw(:,:, i-1);
       end
        for i = ceil(numSteps/2):numSteps
           T_cw(:,:, i) = T_step_r*T_cw(:,:, i-1);
       end
    else
       error('Unknown trajectory type');
    end
end