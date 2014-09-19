function [predWeight] = getPredVectorWeight(predVector, searchObject, predWeights, pipelineOptions)
%GETOBSEDGEINFOMAT Process the model to return the information matrix for
%an image observation
%  if T_wcam(3,3)*T_wcam(1,3) > 0
%        infoMat = (1)^-2*eye(2); %image noise
%  else
%        infoMat = (1)^-2*eye(2);
%  end
% 

%clusterId = getClusterIds(predVector, clusteringModel);
optimalWeight = pipelineOptions.mEstWeight;
% 
%omnicron = optimalWeight/max(clusterWeights);
%

idx = knnsearch(searchObject, predVector', 'K', 5);

predWeight = median(predWeights(idx))/max(optimalWeight)*optimalWeight;

% if clusterId > 0
%     predWeight = omnicron*clusterWeights(clusterId);
% else
%     predWeight = 0.05*optimalWeight;
% end


%[predWeight, ~] = gp(clusteringModel.hyp2, @infExact, [], @covSEiso, @likGauss, clusteringModel.centroids',clusterWeights', predVector');
%predWeight = max(predWeight*omnicron, optimalWeight*0.1); 

end