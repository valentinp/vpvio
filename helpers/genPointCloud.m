function [points] = genPointCloud( x_lim, y_lim, z_lim, pt_number)
points = [x_lim(1) + (x_lim(2)-x_lim(1)).*rand(pt_number,1);
    y_lim(1) + (y_lim(2)-y_lim(1)).*rand(pt_number,1);
    z_lim(1) + (z_lim(2)-z_lim(1)).*rand(pt_number,1)];
end

