function [ R_noisy ] = addNoiseRot( R, noiseLevel)
%ADDNOISEROT Adds noise to the rotation matrix
r = rodrigues(R);
r = r + noiseLevel*randn(3,1);
R_noisy = rodrigues(r);
end

