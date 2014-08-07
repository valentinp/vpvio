function [ matchedToAllIndices ] = simMatchToGlobalLandmarks( matchedRelFeatures, allLandmarkFeatures, allLandmarkPositions_w, pixelMeasurements, K, T_wcam )
%MATCHTOGLOBALLANDMARKS Match local features to a global set using a
%Chi-squared test

        matchedToAllIndices = simMatchFeatures(matchedRelFeatures, allLandmarkFeatures);
        triangPoints_c = homo2cart(inv(T_wcam)*cart2homo(allLandmarkPositions_w(:, matchedToAllIndices(:, 2))));
        simulatedPixels_c = homo2cart(K*triangPoints_c);
        
        %Check image space error vectors
        error_vecs = simulatedPixels_c - pixelMeasurements(:,matchedToAllIndices(:, 1));
        error_norms = sum(error_vecs.^2, 1);
        
        %Cut off at threshold
        inlierIdx = error_norms < 50^2;

        matchedToAllIndices = matchedToAllIndices(inlierIdx,:);
end

