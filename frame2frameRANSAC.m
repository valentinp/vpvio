function [ bestTranslation, bestIdx, inlierIdx] = frame2frameRANSAC(prevPts, currPts, rotMat)
% frame2frameRANSAC Calculates the translation between two frames given a
% rotation matrix
%
%   Inputs:
%   -------
%   prevPts  - 3xN matrix containing 3D feature vectors from the previous
%   frame
%   currPts  - 3xN matrix containing 3D feature vectors from the current
%   frame
%   rotMat - 3x3 rotation matrix prior
%
%   Outputs:
%   --------   
%   bestTranslation - 3x1 vector

    RANSAC_ITERS = 200;
    INLIER_THRESH = 1e-06;

    numPts = size(prevPts, 2);
    bestTranslation = zeros(3,1);
    highestNumInliers = 0;
    bestIdx = [1,2]';
    inlierIdx = 1:numPts;


    for ransac_i = 1:RANSAC_ITERS
        %Select two matched feature vectors at random
        randInts = [0;0];
        while randInts(1) == randInts(2)
            randInts = randi(numPts, [2 1]);
        end
        featVecP1 = prevPts(:, randInts(1));
        featVecC1 = currPts(:, randInts(1));
        featVecP2 = prevPts(:, randInts(2));
        featVecC2 = currPts(:, randInts(2));
        
        %Calculate epipolar plane normals
        n1 = cross(featVecP1, rotMat*featVecC1);
        n2 = cross(featVecP2, rotMat*featVecC2);

        n1 = n1/norm(n1);
        n2 = n2/norm(n2);

        %The raw normal cross product is used
        translation_raw = cross(n1, n2);
        translation = translation_raw/norm(translation_raw);

        %Determine the sign of the translation vector
        if (featVecP1 - rotMat*featVecC1)'*translation < 0
            translation = -translation;
        end
        
        if (sum(isnan(translation)) > 0)
            error('Translation is NaN error.');
        end
        


        %Calculate number of inliers
        


        
        triangPoints = triangulate2(prevPts, currPts, rotMat, translation); 
        P_prev = [eye(3) zeros(3,1); 0 0 0 1];
        P_curr = [rotMat' -rotMat'*translation; 0 0 0 1];
    
        
        reprojections_curr = homo2cart(P_curr*cart2homo(triangPoints));
        reprojections_prev = homo2cart(P_prev*cart2homo(triangPoints));

        reprojections_curr = normalize(reprojections_curr);
        reprojections_prev = normalize(reprojections_prev);

        dotProducts_curr = dot(reprojections_curr, currPts);
        dotProducts_prev = dot(reprojections_prev, prevPts);

        errors_curr = 1 - dotProducts_curr;
        errors_prev = 1 - dotProducts_prev;

        tempInliers = find(errors_curr < INLIER_THRESH & errors_prev < INLIER_THRESH);
        numInliers = length(tempInliers);

        if numInliers > highestNumInliers 
            bestIdx = randInts;
            inlierIdx = tempInliers;
            highestNumInliers = numInliers;
            bestTranslation = translation;
        end
    end
    
end

