function [triangPoints] = triangulate2(vecs_1, vecs_2, C_12, t_21_1)
% triangulate Triangulates 3D points from two sets of feature vectors and a
% a frame-to-frame transformation
%
%   Inputs:
%   -------
%   vecs_1  - 3xN matrix containing unit feature vectors from the previous
%   frame
%   vecs_2  - 3xN matrix containing unit feature vectors from the current
%   frame
%   C_12 - 3x3 rotation matrix from the current to the previous frame
%
%   t_21_1 - 3x1 vector from the previous to the current frame
%   expressed in the previous frame
%
%   Outputs:
%   --------   
%   triangPoints - 3xN matrix containing 3D points expressed in the previous frame

numPts = size(vecs_1, 2);
triangPoints = zeros(3, numPts);

for pt_i = 1:numPts
   %Setup linear least squares problem
   % Refer to pg 45 of the Laurent Kneip PhD pdf
   v_1 = vecs_1(:,pt_i);
   v_2 = vecs_2(:,pt_i);
   
   A = [v_1 -C_12*v_2];
   b = t_21_1;
   
   scalar_consts = abs(A\b);
   
   triangPoints(:, pt_i) = (scalar_consts(1)*v_1 + scalar_consts(2)*C_12*v_2 + t_21_1)/2;


end

end

