function [R_new] = propagateCovariance(xPrev, a, dt, noise_params, RPrev)
%PROPAGATECOVARIANCE Propagate the covariance
%See Leutenegger and Sola 
C_ws = rotmat_from_quat(xPrev.q);

F_c = [zeros(3,3) zeros(3,3) eye(3,3) zeros(3,3) zeros(3,3); 
zeros(3,3) zeros(3,3) zeros(3,3) C_ws  zeros(3,3); 
zeros(3,3)  skew(C_ws*(a - xPrev.b_a)) zeros(3,3) zeros(3,3) -C_ws;
zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3);
zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)];

F_d = eye(15,15) + F_c*dt;


G = [zeros(3, 12); blkdiag(eye(3), eye(3), eye(3), eye(3))];
Q = blkdiag(noise_params.sigma_g^2*eye(3), noise_params.sigma_a^2*eye(3), noise_params.sigma_bg^2*eye(3), noise_params.sigma_ba^2*eye(3));



R_new = F_d*RPrev*F_d' + G*Q*G'*dt;


end

