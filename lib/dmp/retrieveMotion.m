function retr = retrieveMotion(setup, params, mode)
    L = [eye(params.nbDOF) * params.alphaY * params.bettaY, eye(params.nbDOF) * params.alphaY];
       
    x = setup.xIni;
    retr = zeros(params.nbDOF, params.nbData);% dretr = zeros(params.nbDOF, params.nbData); ddretr = zeros(params.nbDOF, params.nbData); exretr = zeros(params.nbDOF, params.nbData); extra = 0;
    dx = zeros(params.nbDOF,1);
    
    for t=1:params.nbData
        % compute acceleration, velocity and position
        if mode == 0
            extra = setup.ft(:,t);
            ddx = L * [setup.xTar-x; -dx] + setup.ft(:,t);
        elseif mode == 1          
            % just compute obstacle_avoidance paramaters the first iteration
            % compute with the desired direction for avoiding an obstacle
%             if t == 1
%                 %dx = params.virtual_dx;
%                 for o = 1:size(setup.obstacles,2)
%                     p_plane = -setup.obstacles(:, o) / norm(setup.obstacles(:, o)); p_plane(1) = 0;
%                     [params.kappa(o), params.beta(o), params.gamma(o)] = h_func.retrieveParameters(setup.geometries(:, o), p_plane); 
%                 end
%             end
            
            extra = avoidObstacle(x, dx, setup.obstacles, params.kappa, params.gamma, params.beta, params.approach);
            if isnan(extra), extra = zeros(size(extra)); end
            ddx = params.tau^2 * L * [setup.xTar-x; -dx] + setup.ft(:,t) + extra;
        end
        dx = dx + ddx * params.dt;
        x = x + dx * params.dt;
        retr(:, t) = x;
        %dretr(:, t) = dx;
        %ddretr(:, t) = ddx;
        %exretr(:, t) = extra;
    end