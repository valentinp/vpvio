function [G] = kinmat_from_rpy(rpy)
% KINMAT_FROM_RPY Kinematical matrix from roll, pitch, yaw Euler angles.
%
%   [G] = KINMAT_FROM_RPY(rpy) computes the Euler kinematic matrix from the
%   current roll, pitch and yaw angles.  This matrix is used to compute the
%   rate of change of the angles as a function of the body frame angular 
%   velocity.
%
%   Note that the matrix is undefined at pitch values of +-90 degrees (due to
%   division by cos(p)).
%
%   Inputs:
%   -------
%    rpy  - 3x1 vector of roll, pitch and yaw Euler angles.
%
%   Outputs:
%   --------
%    G  - 3x3 Euler kinematical matrix.

cr = cos(rpy(1));
sr = sin(rpy(1));
cp = cos(rpy(2));
tp = tan(rpy(2));

G = [1, tp*sr, tp*cr;
     0,    cr,   -sr;
     0, sr/cp, cr/cp];