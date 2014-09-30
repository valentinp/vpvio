function [ newSeeds ] = updateSeeds( oldSeeds, invDepthList, invTau2List)
%UPDATESEEDS Updates inverse depth filter based on 'seeds' (observations of
%depths)
%   oldSeeds is a struct of seeds that have sigma, mu, a and b properties
%   invDepths is a list of inverse depths observations
%   invTau2 is a list of 1/tau^2 variances for the inverse depths

newSeeds = {};

for seed_i = 1:length(oldSeeds)
    seed = oldSeeds(seed_i);
    invTau2 = invTau2List(seed_i);
    invDepth = invDepthList(seed_i);
    
    norm_scale = sqrt(seed.sigma2 + invTau2);
    
     s2 = 1./(1./seed.sigma2 + 1./invTau2);
     m = s2*(seed.mu/seed.sigma2 + invDepth/invTau2);
     C1 = seed.a/(seed.a+seed.b) * normpdf(invDepth,seed.mu,norm_scale);
     C2 = seed.b/(seed.a+seed.b) * 1./seed.z_range;
     normalization_constant = C1 + C2;
      C1 = C1/normalization_constant;
      C2 = C2/normalization_constant;

      f = C1*(seed.a+1)/(seed.a+seed.b+1.) + C2*seed.a/(seed.a+seed.b+1.);
        e = C1*(seed.a+1.)*(seed.a+2.)/((seed.a+seed.b+1.)*(seed.a+seed.b+2.)) ...
          + C2*seed.a*(seed.a+1.0)/((seed.a+seed.b+1.0)*(seed.a+seed.b+2.0));

  mu_new = C1*m+C2*seed.mu;
  
  newSeeds(seed_i).mu = mu_new;
  newSeeds(seed_i).sigma2 = C1*(s2 + m*m) + C2*(seed.sigma2 + seed.mu*seed.mu) - mu_new*mu_new;
  newSeeds(seed_i).a = (e-f)/(f-e/f);
  newSeeds(seed_i).b = seed.a*(1.0-f)/f;
  newSeeds(seed_i).z_range = seed.z_range;
  newSeeds(seed_i).id = seed.id;
  newSeeds(seed_i).trueDepth = seed.trueDepth;
  newSeeds(seed_i).numObs = seed.numObs + 1;
  newSeeds(seed_i).lastPoseKey = seed.lastPoseKey + 1;

end

