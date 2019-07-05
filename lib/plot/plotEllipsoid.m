function h = plotEllipsoid(obstacle_location, obstacle_size)
    t_range = linspace(0, 2*pi);
    u_range = linspace(0, 2*pi);
    [t, u] = meshgrid(t_range, u_range);
    xt = obstacle_size(1)/2 .* cos(t(:)) .* cos(u(:)) + obstacle_location(1);
    yt = obstacle_size(2)/2 .* sin(t(:)) .* cos(u(:)) + obstacle_location(2);
    zt = obstacle_size(3)/2 .* sin(u(:)) + obstacle_location(3);
    h = plot3(xt, yt, zt, 'color', [0.8 0.8 0.8]);
end