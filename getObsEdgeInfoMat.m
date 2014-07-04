function [infoMat] = getObsEdgeInfoMat(T_wcam, pixelMeasurement)
%GETOBSEDGEINFOMAT Process the model to return the information matrix for
%an image observation
%  if T_wcam(3,3)*T_wcam(1,3) > 0
%        infoMat = (1)^-2*eye(2); %image noise
%  else
%        infoMat = (1)^-2*eye(2);
%  end

if pixelMeasurement(2) > 320
    infoMat = (3)^-2*eye(2);
else
    infoMat = (0.1)^-2*eye(2);
end

end

