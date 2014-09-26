function [xUpdate] = integrateIMUMourikis(xPrev, a, omega, dt, noise_params, g_w)
%INTEGRATEIMUSTREAM Integrate IMU stream and return state and covariance
%estimates

  
    %The state is given by:
    %x = [p q v b_g b_a];
    
    omegaBiasRemoved = omega - xPrev.b_g;

    xUpdate.q = quat_product([1; 0.5*omegaBiasRemoved*dt], xPrev.q);
    xUpdate.b_a = xPrev.b_a;
    xUpdate.b_g = xPrev.b_g;
    
    s_l = dt*(rotmat_from_quat(xPrev.q)*(a - xPrev.b_a));
    y_l = s_l*dt;
    
    xUpdate.v = xPrev.v + rotmat_from_quat(xPrev.q)*s_l + g_w*dt;
    xUpdate.p = xPrev.p + xPrev.v*dt + rotmat_from_quat(xPrev.q)*y_l + 0.5*g_w*dt^2;
    


end

