function [na] = angle_normalize(a)
% ANGLE_NORMALIZE Normalize angle to lie in range -pi to pi.
% 
%   [na] = ANGLE_NORMALIZE(a) normalizes all angles in the vector a to lie 
%   in the range -pi < a(i) <= pi.  This is necessary for proper addition and 
%   subtraction etc.
%
%   Inputs:
%   -------
%    a  - nx1 vector of angles, in radians.
%
%   Outputs:
%   --------
%    na  - nx1 vector of normalized angles, in range -pi < na <= pi.

na = mod(a, 2*pi);

idx = na <= -pi;
na(idx) = na(idx) + 2*pi;

idx = na > pi;
na(idx) = na(idx) - 2*pi;