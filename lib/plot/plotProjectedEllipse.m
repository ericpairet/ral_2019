function h = plotProjectedEllipse(obstacle, dx, projected_geometry)
    alpha_range = linspace(0, 2*pi);
    x_slice = projected_geometry(1)/2 .* cos(alpha_range(:));
    y_slice = projected_geometry(2)/2 .* sin(alpha_range(:));
    z_slice = zeros(size(y_slice));
    gamma = atan2(dx(3), dx(2));
    projected_ellipse = rotx(rad2deg(gamma)) * [x_slice y_slice z_slice]';
    h = plot3(projected_ellipse(1, :) + obstacle(1), projected_ellipse(2, :) + obstacle(2), projected_ellipse(3, :) + obstacle(3), 'r-', 'LineWidth', 3); 
end