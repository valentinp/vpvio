function [ taus ] = computeTaus(T_ref_curr, featVec_curr, depths, K)
%COMPUTETAU Computes depth uncertainty deviations (taus) from image uncertainty

    featNum = size(featVec_curr, 2);
    taus = zeros(1, featNum);
    pxNoise = 1;
    focalLength = K(1,1);
    pxErrorAngle = atan(pxNoise/(2*focalLength))*2;
    
    for f_i = 1:featNum
        t = T_ref_curr(1:3,4);
        a = featVec_curr(:, f_i)*depths(f_i) - t;
        alpha = acos(dot(featVec_curr(:,f_i), t)/norm(t));
        beta = acos(dot(-t, a)/(norm(t)*norm(a)));
        
        betaWithError = beta + pxErrorAngle;
        gammaWithError = pi - alpha - betaWithError; %Interior angles of triangle sum to pi
        
        depthWithError = norm(t)*sin(betaWithError)/sin(gammaWithError); %Using Sine Law
        
        taus(f_i) = depthWithError - depths(f_i);
        
    end
    
end

