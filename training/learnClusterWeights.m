function clusterWeights = learnClusterWeights(pixelMeasurements, rgbimage, T_rc_gt,matchedReferenceUnitVectors, matchedCurrentUnitVectors, T_rcam, clusteringModel)
%LEARNCLUSTERWEIGHTS Learns the weights on clusters within the prediction
%space
    predVectors = computePredVectors(pixelMeasurements, rgbimage);
    clusterIds = getClusterIds(predVectors, clusteringModel);

    %Initialize the weights
    clusterWeights = ones(1, clusteringModel.clusterNum);
    
    %For each cluster, generate error vector and resulting weight
    for c_id = 1:clusteringModel.clusterNum
        
        %Ensure there are enough points in this cluster
        if sum(clusterIds == c_id) < 5
            continue;
        end
        
        %Extract the unit vectors associated with this cluster
        refUV = matchedReferenceUnitVectors(:, clusterIds == c_id);
        curUV = matchedCurrentUnitVectors(:, clusterIds == c_id);
        
        T_rc_est = opengv('rel_nonlin_central',double(1:size(refUV,2)), double(refUV), double(curUV), double(T_rcam(1:3,1:4)));
        p_rc_c_est = T_rc_est(:, 4);
        
        %Ground truth
        p_rc_c_gt = homo2cart(T_rc_gt(:, 4));
        
        %The cosine distance defines the error
        error = acosd(dot(p_rc_c_gt, p_rc_c_est)/(norm(p_rc_c_gt)*norm(p_rc_c_est)));
        
        %German-Maclure weight
        clusterWeights(c_id) = 100/(1+error^2)^2;
        
    end

end

