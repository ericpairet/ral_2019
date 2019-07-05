function drawLocalReferenceFrame(origin, x_axis, y_axis, z_axis, length)
    h = quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 0);
    set(h, 'Color', 'r', 'LineWidth', 40 * length + 1, 'MaxHeadSize', 4, 'ShowArrowHead', 'off')
    set(h, 'ShowArrowHead', 'on')

    h = quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 0);
    set(h, 'Color', 'g', 'LineWidth', 40 * length + 1, 'MaxHeadSize', 4, 'ShowArrowHead', 'off')
    set(h, 'ShowArrowHead', 'on')
    
    h = quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 0);
    set(h, 'Color', 'b', 'LineWidth', 40 * length + 1, 'MaxHeadSize', 4, 'ShowArrowHead', 'off')
    set(h, 'ShowArrowHead', 'on')