function params = loadParams()
    % DMP params
    params.nbDOF = 3; % demo only for nbDOF = 3
    params.nbRBF = 35;
    params.alpha = 0.5; % decay_time parameter
    params.nbData = 500;
    params.dt = 0.005;
    params.kP = 20; params.kV = 10;
    params.alphaY = params.kV; params.bettaY = params.alphaY / 4;
    params.polDeg = 2;
    params.tau = 1;
    
    % OA params
    params.approach = 1; % 0 - original | 1 - proposal
    params.clearance = 0.15;
end