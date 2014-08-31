function [V] = calcLandmarkUncertainty(prevPx, currPx, pixelNoiseMat, b, K)
%CALCLANDMARKUNCERTAINTY Propagates pixel uncertainty into 3D space
f_x = K(1,1);
f_y = K(2,2);
c_x = K(1,3);
c_y = K(2,3);

u_l = prevPx(1);
v_l = prevPx(2);
u_r = currPx(1);
v_r = currPx(2);


G = b/(u_l - u_r)^2*[-u_r+c_x, 0, u_l - c_x, 0; ...
                                  -(f_x/f_y*(0.5*(v_l +v_r) - c_y)), 0.5*f_x/f_y, f_x/f_y*(0.5*(v_l + v_r) - c_y), 0.5*f_x/f_y; ...
                                 -f_x, 0, f_x 0];
V = G*pixelNoiseMat*G';
                             
end

