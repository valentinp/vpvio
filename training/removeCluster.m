function locationsSansCluster = removeCluster(locations, predVectors, centroid, threshDist)
%REMOVECLUSTER Remove all keypoints that fall within a certain threshold

distances = vecNorms(predVectors - repmat(centroid, [1 size(predVectors,2)])).^2;

locationsSansCluster = locations;
locationsSansCluster(:, distances < threshDist) = [];

end

