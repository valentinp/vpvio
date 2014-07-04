function [ T_out ] = downSampleTransform(T_in, n)
%DOWNSAMPLETRANSFORM Keep every nth transform
%   T_in must be 4x4xN
%   T_out is 4x4xM where M is floor(N/n)
    idx = 1:n:size(T_in,3);
    T_out = T_in(:,:, idx);
end

