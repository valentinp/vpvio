function [inlierIdx] = findInliers(prevPts, currPts, R_rcam, p_camr_r, pixelMeasurements_c, K, pipelineOptions)
% findInliers Finds inliers from unit measurement vectors based on a
% threshold reprojection test
        
        T_camr = [R_rcam' -R_rcam'*p_camr_r; 0 0 0 1];
        
        triangPoints_r = triangulate(prevPts, currPts, R_rcam, p_camr_r); 
        triangPoints_c = homo2cart(T_camr*cart2homo(triangPoints_r));
       
        simulatedPixels_c = homo2cart(K*triangPoints_c);
        %simulatedPixels_r = homo2cart(K*R_rcam*triangPoints_c);
        
        %Check disparity
        %disparity = simulatedPixels_c(1,:) - simulatedPixels_r(1,:);
        %Check image space error vectors
        error_vecs = simulatedPixels_c - pixelMeasurements_c;
        error_norms = sum(error_vecs.^2, 1);
       
        inlierIdx = find(error_norms < pipelineOptions.inlierThreshold);
end

