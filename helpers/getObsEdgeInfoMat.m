function [infoMat] = getObsEdgeInfoMat(predVector, clusteringModel, clusterWeights)
%GETOBSEDGEINFOMAT Process the model to return the information matrix for
%an image observation
%  if T_wcam(3,3)*T_wcam(1,3) > 0
%        infoMat = (1)^-2*eye(2); %image noise
%  else
%        infoMat = (1)^-2*eye(2);
%  end

clusterId = getClusterIds(predVector, clusteringModel);

%Pre-process the weights
% mW = min(clusterWeights);
% clusterWeights = clusterWeights - mW;
% clusterWeights = (clusterWeights + 1);

%     switch clusterId 
% 
%         case 0
%           infoMat = 0.1*eye(2);
%         case 1
%             if clusterWeights(1) > clusterWeights(2)
%                infoMat = eye(2);
%             else
%                infoMat = 0.1*eye(2);
%             end
%         case 2
%             if clusterWeights(1) < clusterWeights(2)
%                infoMat = eye(2);
%             else
%                infoMat = 0.1*eye(2);
%             end
%     end
%   if clusterId > 0
%   infoMat = clusterWeights(clusterId)*eye(2);
%   else
%   infoMat = eye(2);
%   end
[weightPred, ~] = gp(clusteringModel.hyp2, @infExact, [], @covSEiso, @likGauss, clusteringModel.centroids',clusterWeights', predVector');
infoMat = max(weightPred, 0)*eye(2); 
end