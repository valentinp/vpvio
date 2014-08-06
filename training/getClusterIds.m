function clusterIds = getClusterIds(predVectors, clusteringModel)
%GETCLUSTERIDS Associated points in the prediction space with each cluster

clusterIds = zeros(1, size(predVectors,2));

%Iterate through clusters and assign integers to points
%If a point does not fall within any cluster, it is assigned a 0
for c_id = 1:clusteringModel.clusterNum
    centroid = clusteringModel.centroids(:, c_id);
    distances = vecNorms(predVectors - repmat(centroid, [1 size(predVectors,2)])).^2;
    clusterIds(distances < clusteringModel.threshDists(c_id)) = c_id;
end

end

