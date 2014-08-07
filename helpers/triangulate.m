function [outPoints] = triangulate(prevPts, currPts, rotMat, translation )
% triangulate Triangulates 3D points from two sets of feature vectors and a
% a frame-to-frame transformation
%
%   Inputs:
%   -------
%   prevPts  - 3xN matrix containing unit feature vectors from the previous
%   frame
%   currPts  - 3xN matrix containing unit feature vectors from the current
%   frame
%   rotMat - 3x3 rotation matrix from the current to the previous frame
%
%   translation - 3x1 vector from the previous to the current frame
%   expressed in the previous frame
%
%   Outputs:
%   --------   
%   outPoints - 3xN matrix containing 3D points expressed in the previous frame

numPts = size(prevPts, 2);
outPoints = zeros(3, numPts);


for pt_i = 1:numPts
   %Setup linear least squares problem
   % Refer to pg 45 of the Laurent Kneip PhD pdf
   f_x_prev = prevPts(1,pt_i);
   f_y_prev = prevPts(2,pt_i);
   f_z_prev = prevPts(3,pt_i);
   
   f_x_curr = currPts(1,pt_i);
   f_y_curr = currPts(2,pt_i);
   f_z_curr = currPts(3,pt_i);
   
   P_pp = [eye(3) zeros(3,1)];
   P_cp = [rotMat' -rotMat'*translation];
   
   A = [f_x_prev*[0 0 1]*P_pp - f_z_prev*[1 0 0]*P_pp;
       f_y_prev*[0 0 1]*P_pp - f_z_prev*[0 1 0]*P_pp;
       f_x_curr*[0 0 1]*P_cp - f_z_curr*[1 0 0]*P_cp;
       f_y_curr*[0 0 1]*P_cp - f_z_curr*[0 1 0]*P_cp;
   ];
   
    %b = -A(:,4);
    %A = A(:,1:3);
    
    
    %Solve LLS with SVD
    %outPoints(:,pt_i) = calcLLSwSVD(A, b);
    %outPoints(:,pt_i) = A\b;
    try 
      [~,~,V] = svd(A);
      outPoints(:,pt_i) = [V(1,4); V(2,4); V(3,4)]/V(4,4);
    catch
        disp('SVD not working.');
    end
    
end

end

