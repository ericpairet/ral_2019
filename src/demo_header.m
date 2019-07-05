function h_func = demo_header()  
    h_func.continuousQueries = @continuousQueries;
end

function continuousQueries(RC, setup, state, params)  
    % some checks
    if ((params.nbDOF ~= length(setup.xIni)) || (params.nbDOF ~= length(setup.xTar))); fprintf(2,'\n setup error: params.nbDOF does not correspond to xIni or xTar \n'); return; end
    if ((setup.xTar(1) - setup.xIni(1)) < 0.6); fprintf(2,'\n setup error: baseline.x (xTar.x - xIni.x) must be larger than the maximum obstacle width (with 0.1 margin) \n'); return; end    

    % set up all figure handlers and static plots
    h_list = createScenario(setup);

    % create baseline
    ud.demo = [linspace(setup.xIni(1), setup.xTar(1), 20); linspace(setup.xIni(2), setup.xTar(2), 20); linspace(setup.xIni(3), setup.xTar(3), 20)];
    [ud.training_data, ud.forcing_term, ud.decay_term] = processDemonstration(ud.demo, params);
    [~, ~, ~, setup.ft] = fitDMP(ud.forcing_term, ud.decay_term, initRBFTimeBased(ud.decay_term, params));
    
    % test random setups
    while true
        tic
        % expand/contract obstacles
        for o = 1 : setup.nbObstacles
            state.obs_x_idx(o) = state.obs_x_idx(o) + state.obs_x_dir(o); if (state.obs_x_idx(o) == 0 || state.obs_x_idx(o) > length(state.obs_x_range)); state.obs_x_dir(o) = -state.obs_x_dir(o); state.obs_x_idx(o) = state.obs_x_idx(o) + state.obs_x_dir(o); end
            state.obs_y_idx(o) = state.obs_y_idx(o) + state.obs_y_dir(o); if (state.obs_y_idx(o) == 0 || state.obs_y_idx(o) > length(state.obs_y_range)); state.obs_y_dir(o) = -state.obs_y_dir(o); state.obs_y_idx(o) = state.obs_y_idx(o) + state.obs_y_dir(o); end
            state.obs_z_idx(o) = state.obs_z_idx(o) + state.obs_z_dir(o); if (state.obs_z_idx(o) == 0 || state.obs_z_idx(o) > length(state.obs_z_range)); state.obs_z_dir(o) = -state.obs_z_dir(o); state.obs_z_idx(o) = state.obs_z_idx(o) + state.obs_z_dir(o); end

            state.obs_w_idx(o) = state.obs_w_idx(o) + state.obs_w_dir(o); if (state.obs_w_idx(o) == 0 || state.obs_w_idx(o) > length(state.obs_w_range)); state.obs_w_dir(o) = -state.obs_w_dir(o); state.obs_w_idx(o) = state.obs_w_idx(o) + state.obs_w_dir(o); end
            state.obs_d_idx(o) = state.obs_d_idx(o) + state.obs_d_dir(o); if (state.obs_d_idx(o) == 0 || state.obs_d_idx(o) > length(state.obs_d_range)); state.obs_d_dir(o) = -state.obs_d_dir(o); state.obs_d_idx(o) = state.obs_d_idx(o) + state.obs_d_dir(o); end
            state.obs_h_idx(o) = state.obs_h_idx(o) + state.obs_h_dir(o); if (state.obs_h_idx(o) == 0 || state.obs_h_idx(o) > length(state.obs_h_range)); state.obs_h_dir(o) = -state.obs_h_dir(o); state.obs_h_idx(o) = state.obs_h_idx(o) + state.obs_h_dir(o); end

            setup.obstacles(:, o) = [state.obs_x_range(state.obs_x_idx(o)); state.obs_y_range(state.obs_y_idx(o)); state.obs_z_range(state.obs_z_idx(o))];
            setup.geometries(:, o) = [state.obs_w_range(state.obs_w_idx(o)); state.obs_d_range(state.obs_d_idx(o)); state.obs_h_range(state.obs_h_idx(o))];
        end

        % dummy p_plane definition
        setup.p_plane = -normalize(setup.obstacles - setup.xIni, 1, 'norm'); setup.p_plane(1, :) = 0;

        % retrieve relevant geometry
        setup.projected_geometry = zeros(2, setup.nbObstacles);
        for o = 1 : setup.nbObstacles
            setup.projected_geometry(:, o) = getProjectedEllipse(setup.geometries(:, o), setup.p_plane(:, o));
        end

        % regualte action level
        input = [setup.projected_geometry(1, :); setup.projected_geometry(2, :); repmat(params.clearance, 1, setup.nbObstacles)];
        params.kappa = RC.NN_12c_k(input);
        params.beta = RC.NN_12ck_b([input; params.kappa]);
        params.gamma = RC.NN_12ckb_g([input; params.kappa; params.beta]);

        % dummy scale time and retrieve trajectory
        params.tau = ((norm(setup.xTar - setup.obstacles) + norm(setup.obstacles - setup.xIni)) + 2 * (max(setup.geometries(:)) + params.clearance)) / norm(setup.xTar - setup.xIni);
        retrieved_trajectory = retrieveMotion(setup, params, 1);

        % refresh scenario
        h_list = updateScenario(setup, retrieved_trajectory, h_list);
        drawnow limitrate nocallbacks
    end
end

function h_list = createScenario(setup)
    % basic plot setup
    h_new = figure(); set(gcf, 'color', 'white')
    h_list.h_task = axes('Parent', h_new); hold(h_list.h_task, 'on');

    axis equal; xlim(h_list.h_task, [-0.1 setup.xTar(1)+0.1]); ylim(h_list.h_task, [-0.5 0.5]); zlim(h_list.h_task, [-0.5 0.5]); 
    camera_view = get(h_list.h_task, 'view'); view(h_list.h_task, camera_view(1), camera_view(2));
    view(-150, 10); set(gca, 'FontSize', 20); view(-27, 10)
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')

    % start and goal
    plot3(h_list.h_task, setup.xIni(1), setup.xIni(2), setup.xIni(3), 'o', 'MarkerSize', 10); text(setup.xIni(1)-0.05, setup.xIni(2), setup.xIni(3)-0.05, '  start', 'FontSize', 20)
    plot3(h_list.h_task, setup.xTar(1), setup.xTar(2), setup.xTar(3), 'o', 'MarkerSize', 10); text(setup.xTar(1)-0.05, setup.xTar(2), setup.xTar(3)-0.05, '  goal', 'FontSize', 20)

    % local frame
    length = 0.1;
    x_axis = setup.xTar - setup.xIni; x_axis = length * x_axis / norm(x_axis); z_axis = [0 0 length];
    y_axis = cross(z_axis, x_axis); y_axis = length * y_axis / norm(y_axis);
    drawLocalReferenceFrame(setup.xIni, x_axis, y_axis, z_axis, length);
    
    % create fake handlers
    h_list.h_traj = plot(0, 0);
    for o = 1 : setup.nbObstacles; h_list.h_obs(o) = plot(0, 0); h_list.h_des(o) = plot(0, 0); end
end

function h_list = updateScenario(setup, x, h_list)
    % delete obstacles and projected geometries
    delete(h_list.h_traj)
    for o = 1 : setup.nbObstacles; delete(h_list.h_obs(o)); delete(h_list.h_des(o)); end

    % plot obstacles and projected geometries
    for o = 1 : setup.nbObstacles
        h_list.h_obs(o) = plotEllipsoid(setup.obstacles(:, o), setup.geometries(:, o));
        h_list.h_des(o) = plotProjectedEllipse(setup.obstacles(:, o), setup.p_plane(:, o), setup.projected_geometry(:, o));
    end

    % plot trajectory
    h_list.h_traj = plot3(h_list.h_task, x(1, :), x(2, :), x(3, :), 'b', 'LineWidth', 2);
end

function projected_geometry = getProjectedEllipse(geometry, dx)
    % get lambdas (half-axis lengths)
    lambdas = [geometry(1)/2 geometry(2)/2 geometry(3)/2];

    alpha = pi/2;
    gamma_normalised = atan2(1/lambdas(3)*dx(3), 1/lambdas(2)*dx(2));
    x = lambdas(1) .* cos(alpha) .* cos(gamma_normalised);
    y = lambdas(2) .* sin(alpha) .* cos(gamma_normalised);
    z = lambdas(3) .* sin(gamma_normalised);

    dist = sqrt(x^2 + y^2 + z^2);
    projected_geometry = 2 * [lambdas(1) dist]'; % get back to overall size
end