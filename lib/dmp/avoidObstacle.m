function pval = avoidObstacle(y, dy, obstacles, kappa, gamma, beta, approach)
    nbDOF = size(y, 1);
    pval = zeros(nbDOF, 1);
    
    % initial checks
    if (norm(dy) < 1E-05); return; end
    
    % compute repulsive force for each obstacle
    for o = 1 : size(obstacles, 2)
        obstacle = obstacles(:, o);

        phi = acos((obstacle - y)' * dy / (norm(obstacle - y) * norm(dy)));
        %fprintf('phi: %d \n', phi)

        % 0 - original
        % 1 - proposed
        if ~approach
            % Biologically-inspired dynamical systems for movement generation: automatic real-time goal adaptation and obstacle avoidance
            % desired steering velocity, phi is always positive, so it always turns right, the cross product below defines where to turn
            dphi = gamma(o) * phi * exp(-beta(o) * phi);
        else
            % proposed obstacle avoidance
            direction = sign(phi);
            if direction == 0; direction = 1; end
            dphi = gamma(o) * direction * exp(-(phi / beta(o)).^2);
        end
        %fprintf('dphi: %d \n', phi)

        % extra term for distance
        if kappa(o) > 0; dphi = dphi .* exp(-kappa(o) * norm(obstacle - y).^2); end

        % perpendicular angle to plane defined by (obstacle - y) and dy
        if nbDOF == 2
            rot_vec = cross([(obstacle-y); 0], [dy; 0]);
        else
            rot_vec = cross((obstacle-y), dy);
        end 
    
        if approach == 1
            if sign(phi) == 0
                rot_vec = [0 0 1];         
            end
        end
        
        if norm(rot_vec) ~= 0
            rot_vec = rot_vec / norm(rot_vec);
        end
        
        % rotation matrix
        % https://math.stackexchange.com/questions/142821/matrix-for-rotation-around-a-vector
        % Rodrigues' Rotation Formula
        rot_angle = pi/2;
        W = [0 -rot_vec(3) rot_vec(2);
             rot_vec(3) 0 -rot_vec(1);
             -rot_vec(2) rot_vec(1) 0];
        R = eye(3) + sin(rot_angle) * W + 2 * (sin(rot_angle / 2)^2) * W^2;
            
        % force to avoid current obstacle
        p = R(1:nbDOF, 1:nbDOF) * dy * dphi;
        pval = pval + p;
    end
end