function [distTravelled] = integrateVelocity(baseDir, dtList)
% Computes the cumulative distance travelled at a given time step using the
% trapezoid rule to integrate velocities
oxts = loadOxtsliteData(baseDir);

distTravelled = zeros(length(oxts), 1);

for i=2:length(oxts)
  
  vf = oxts{i}(9); % fw
  vl = oxts{i}(10); % left
  vu = oxts{i}(11); % up
  
  %Previous values  
  vf_p = oxts{i-1}(9); 
  vl_p = oxts{i-1}(10); 
  vu_p = oxts{i-1}(11); 
  
  dt = abs(dtList(i));
  
  
  %Use the trapezoid rule
  
  df = 0.5*(dt)*(vf + vf_p);
  dl = 0.5*(dt)*(vl + vl_p);
  du = 0.5*(dt)*(vu + vu_p);
  
  %Compute distance travelled
  distTravelled(i) = distTravelled(i-1) + norm([df dl du]);
end

