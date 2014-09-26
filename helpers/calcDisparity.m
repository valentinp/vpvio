function [meanDisp] = calcDisparity(prevPix, currPix, R_rcam, K)
% findInliers Finds inliers from unit measurement vectors based on a
% threshold reprojection test

        %T_camr = [R_rcam' -R_rcam'*p_camr_r; 0 0 0 1];

        %triangPoints_r = triangulate2(prevPts, currPts, R_rcam, p_camr_r); 
        %triangPoints_c = homo2cart(T_camr*cart2homo(triangPoints_r));
        
        %simulatedPixels_c = homo2cart(K*triangPoints_c);
        %simulatedPixels_r = homo2cart(K*R_rcam*triangPoints_c);
        
        %simulatedPixels_c = invK*currPts.*repmat(1./currPts(3,:), [3 1]);
        %simulatedPixels_r = invK*prevPts.*repmat(1./prevPts(3,:), [3 1]);
        
        currVectors = R_rcam*inv(K)*cart2homo(currPix);
        currPixUnRot = homo2cart(K*currVectors);
        
        disparity = sqrt(abs(prevPix(1,:) - currPixUnRot(1,:)).^2 + abs(prevPix(2,:) - currPixUnRot(2,:)).^2);
        meanDisp = median(disparity);
    
end

