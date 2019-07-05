function [setup, state] = loadSetup()
    % scenario definition
    setup.xIni = [0.0 0.0 0.0]';
    setup.xTar = [2.0 0.0 0.0]';
    setup.nbObstacles = 5;

    % obstacle location (xyz) and size (whd) range
    state.obs_x_range = 0.3:0.01:(setup.xTar(1)-0.3); % with maximum obstacle size of 0.5m, leave 5 centrimeters for convergence and trajectory start
    state.obs_y_range = -0.2:0.01:0.2;
    state.obs_z_range = -0.2:0.01:0.2;

    state.obs_w_range = 0.05:0.01:0.5;
    state.obs_h_range = 0.05:0.01:0.5;
    state.obs_d_range = 0.05:0.01:0.5;

    % initialise obstacle indices
    setup.obstacles = zeros(length(setup.xIni), setup.nbObstacles);
    setup.geometries = zeros(length(setup.xIni), setup.nbObstacles);
    for o = 1 : setup.nbObstacles
        state.obs_x_idx(o) = randi(length(state.obs_x_range)); state.obs_x_dir(o) = 2 * (rand() < 0.5) - 1; 
        state.obs_y_idx(o) = randi(length(state.obs_y_range)); state.obs_y_dir(o) = 2 * (rand() < 0.5) - 1; 
        state.obs_z_idx(o) = randi(length(state.obs_z_range)); state.obs_z_dir(o) = 2 * (rand() < 0.5) - 1; 

        state.obs_w_idx(o) = randi(length(state.obs_w_range)); state.obs_w_dir(o) = 2 * (rand() < 0.5) - 1; 
        state.obs_d_idx(o) = randi(length(state.obs_d_range)); state.obs_d_dir(o) = 2 * (rand() < 0.5) - 1; 
        state.obs_h_idx(o) = randi(length(state.obs_h_range)); state.obs_h_dir(o) = 2 * (rand() < 0.5) - 1;

        setup.obstacles(:, o) = [state.obs_x_range(state.obs_x_idx(o)); state.obs_y_range(state.obs_y_idx(o)); state.obs_z_range(state.obs_z_idx(o))];
        setup.geometries(:, o) = [state.obs_w_range(state.obs_w_idx(o)); state.obs_d_range(state.obs_d_idx(o)); state.obs_h_range(state.obs_h_idx(o))];
    end
end