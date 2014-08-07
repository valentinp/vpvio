function [infoMat] = getObsEdgeInfoMat(predVector, clusteringModel, clusterWeights)
%GETOBSEDGEINFOMAT Process the model to return the information matrix for
%an image observation
%  if T_wcam(3,3)*T_wcam(1,3) > 0
%        infoMat = (1)^-2*eye(2); %image noise
%  else
%        infoMat = (1)^-2*eye(2);
%  end

clusterId = getClusterIds(predVector, clusteringModel);

if clusterId > 0
    infoMat = clusterWeights(clusterId)*eye(2);
else
    infoMat = eye(2);
end

% if pixelMeasurement(2) > 320
%     infoMat = (5)^-2*eye(2);
% else
%     infoMat = (0.05)^-2*eye(2);
% end

end

