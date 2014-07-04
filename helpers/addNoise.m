function noisyBearingVectors = addNoise(bearingVectors,focal_length,pixel_noise)

    %v_noisy = v_clean + pixel_noise*randn(size(v_clean));
    noisyBearingVectors = zeros(size(bearingVectors));
    for i=1:size(bearingVectors, 2)
    %find good vector in normal plane based on good conditioning
    inPlaneVector1 = zeros(3,1);
    v_clean = bearingVectors(:,i);
    if v_clean(1,1) > v_clean(2,1) && v_clean(1,1) > v_clean(3,1)
        inPlaneVector1(2,1) = 1.0;
        inPlaneVector1(3,1) = 0.0;
        inPlaneVector1(1,1) = 1.0/v_clean(1,1) * (-v_clean(2,1));
    else
        if v_clean(2,1) > v_clean(3,1) && v_clean(2,1) > v_clean(1,1)
            inPlaneVector1(3,1) = 1.0;
            inPlaneVector1(1,1) = 0.0;
            inPlaneVector1(2,1) = 1.0/v_clean(2,1) * (-v_clean(3,1));
        else
            %v_clean(3,1) > v_clean(1,1) && v_clean(3,1) > v_clean(2,1)
            inPlaneVector1(1,1) = 1.0;
            inPlaneVector1(2,1) = 0.0;
            inPlaneVector1(3,1) = 1.0/v_clean(3,1) * (-v_clean(1,1));
        end
    end
    
    %normalize the in-plane vector
    inPlaneVector1 = inPlaneVector1 / norm(inPlaneVector1);
    inPlaneVector2 = cross(v_clean,inPlaneVector1);
    
    noiseX = pixel_noise * (rand-0.5)*2.0;% / sqrt(2);
    noiseY = pixel_noise * (rand-0.5)*2.0;% / sqrt(2);
    
    v_noisy = focal_length * v_clean + noiseX * inPlaneVector1 + noiseY * inPlaneVector2;
    
    v_noisy_norm = norm(v_noisy);
    v_noisy = v_noisy ./ v_noisy_norm;
    noisyBearingVectors(:,i) = v_noisy;
    end
    
end