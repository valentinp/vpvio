function [points] = genLandmarks( x_lim, y_lim, z_lim, pt_number)
points = [x_lim(1) + (x_lim(2)-x_lim(1)).*rand(1, pt_number);
    y_lim(1) + (y_lim(2)-y_lim(1)).*rand(1, pt_number);
    z_lim(1) + (z_lim(2)-z_lim(1)).*rand(1, pt_number)];
end

