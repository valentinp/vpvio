function [x_stream, P_stream] = integrateIMUStream(a_stream, omega_stream, dt_stream, x_0, noise_params, g_w)
%INTEGRATEIMUSTREAM Integrate IMU stream and return state and covariance
%estimates

    %     %Extract sigma parameters
    %     sigma_g = noise_params.sigma_g;
    %     sigma_a = noise_params.sigma_a;
    %     sigma_bg = noise_params.sigma_bg;
    %     sigma_ba = noise_params.sigma_ba;
    %     tau = noise_params.tau;

    numMeasurements = size(a_stream,2);
    
    %The state is given by:
    %x = [p q v b_g b_a];
    
    P_stream = {};
    x_stream = {};
    
    for iter = 1:numMeasurements
           if iter == 1
               xPrev = x_0;
           else
               xPrev = x_stream{iter-1};
           end
           
           %Calculate xDot
           w = getWhiteNoise(noise_params);
           
           %Select the time delta
           dt = dt_stream(iter);
           
            %RK4 Integration
            % Given x' = f(x), RK4 integration proceeds as follows:
            % x(n+1) = x(n)+dt/6*(k1+2*k2+2*k3+k4); where
            % k1 = f(x); 
            % k2 = f(x+dt/2*k1)
            % k3 = f(x+dt/2*k2)
            % k4 = f(x+dt*k3)
           
            k = {};
            
            k_coeffs = [1 0.5 0.5 1];
            b_coeffs = [1/6 1/3 1/3 1/6]; %RK4
            %b_coeffs = [1 0 0 0]; %Forward Euler
          
            k{1} = getxDot(xPrev, omega_stream(:,iter), a_stream(:,iter), w, noise_params, g_w);
           
            for i_k = 2:4
                xNew.p = xPrev.p + (k{i_k-1}.p)*k_coeffs(i_k)*dt;
                xNew.q = xPrev.q + (k{i_k-1}.q)*k_coeffs(i_k)*dt; %Possibly renormalize here
                xNew.v = xPrev.v + (k{i_k-1}.v)*k_coeffs(i_k)*dt;
                xNew.b_g = xPrev.b_g + (k{i_k-1}.b_g)*k_coeffs(i_k)*dt;
                xNew.b_a = xPrev.b_a + (k{i_k-1}.b_a)*k_coeffs(i_k)*dt;
                k{i_k} = getxDot(xNew, omega_stream(:,iter), a_stream(:,iter), w, noise_params, g_w);
            end
            
           xIter.p = xPrev.p + dt*(b_coeffs(1)*k{1}.p + b_coeffs(2)*k{2}.p + b_coeffs(3)*k{3}.p + b_coeffs(4)*k{4}.p);
           xIter.v = xPrev.v + dt*(b_coeffs(1)*k{1}.v + b_coeffs(2)*k{2}.v + b_coeffs(3)*k{3}.v + b_coeffs(4)*k{4}.v);
           xIter.b_g = xPrev.b_g + dt*(b_coeffs(1)*k{1}.b_g + b_coeffs(2)*k{2}.b_g + b_coeffs(3)*k{3}.b_g + b_coeffs(4)*k{4}.b_g);
           xIter.b_a = xPrev.b_a + dt*(b_coeffs(1)*k{1}.b_a + b_coeffs(2)*k{2}.b_a + b_coeffs(3)*k{3}.b_a + b_coeffs(4)*k{4}.b_a);
           xIter.q = (xPrev.q + dt*(b_coeffs(1)*k{1}.q + b_coeffs(2)*k{2}.q + b_coeffs(3)*k{3}.q + b_coeffs(4)*k{4}.q)); %Note the renormalization
           %xIter.q = q_stream(:, iter);
           x_stream{iter} = xIter;
          
          

    end
    
    %Non linear kinematics
    % x - state
    % w - noise vector
    % omega, a - ang. rates and accels.
    % tau - time constant for accelerometer bias
    function xdot = getxDot(x, omega, a, w, noise_params, g_w)
            xdot.p = x.v;
%             xdot.q = 0.5*omegaMat(omega, w.w_g, x.b_g)*x.q;
%             xdot.v = rotmat_from_quat(quat_normalize(x.q))*(a + w.w_a - x.b_a); %% + [0; 0; 9.804];
%             xdot.b_g = w.w_bg;
%             xdot.b_a = -1/noise_params.tau*x.b_a + w.w_ba;
             xdot.q = 0.5*omegaMat(omega, zeros(3,1), x.b_g)*x.q;
             xdot.v = rotmat_from_quat(quat_normalize(x.q))*(a - x.b_a) + g_w; 
             xdot.b_g = zeros(3,1);
             xdot.b_a = zeros(3,1);

    end

    %White noise vector
    function w = getWhiteNoise(sigma_params)
        w.w_g = sigma_params.sigma_g.*randn(3,1);
        w.w_a = sigma_params.sigma_a.*randn(3,1);
        w.w_bg = sigma_params.sigma_bg.*randn(3,1);
        w.w_ba = sigma_params.sigma_ba.*randn(3,1);
    end
 %Quaternion kinematic matrix
    function bigOmega = omegaMat(omega_meas, w_g, b_g)
        %Is it +w_g or -w_g? Does it matter?
        omega_true = omega_meas + w_g - b_g;
        ox = omega_true(1);
        oy = omega_true(2);
        oz = omega_true(3);
        bigOmega = [0 -ox -oy -oz; ...
                    ox 0 oz -oy; 
                    oy -oz 0 ox; 
                    oz oy -ox 0;];
        %See technical note on quaternions by Sola. Page 6.
        %This formulation assumes that the quaternion has the scalar part
        %at the start
    end

end

