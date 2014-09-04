function [predWeight] = getPredVectorWeight(predVector, clusteringModel, clusterWeights)
%GETOBSEDGEINFOMAT Process the model to return the information matrix for
%an image observation
%  if T_wcam(3,3)*T_wcam(1,3) > 0
%        infoMat = (1)^-2*eye(2); %image noise
%  else
%        infoMat = (1)^-2*eye(2);
%  end
% 
% clusterId = getClusterIds(predVector, clusteringModel);
optimalWeight = 1;
% 
medianWeight = max(clusterWeights);
omnicron = optimalWeight/medianWeight;
% 
% if clusterId > 0
%     predWeight = omnicron*clusterWeights(clusterId);
% else
%     predWeight = optimalWeight*0.1;
% end
[predWeight, ~] = gp(clusteringModel.hyp2, @infExact, [], @covSEiso, @likGauss, clusteringModel.centroids',clusterWeights', predVector');
predWeight = max(predWeight, optimalWeight*0.1)*omnicron; 

end