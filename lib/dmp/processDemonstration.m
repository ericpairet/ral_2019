function [training_data, forcing_term, decay_term] = processDemonstration(demostration, params)
    training_data = [];
    forcing_term = [];
    decay_term = [];
    
    % dynamic index of position, velocity and acceleration
    posId = [1:params.nbDOF]; velId = [params.nbDOF+1:2*params.nbDOF]; accId = [2*params.nbDOF+1:3*params.nbDOF]; 
    
    % compute decay term
    decay_term(1) = 1;
    for t=2:params.nbData
        decay_term(t) = decay_term(t-1) - params.tau * params.alpha * decay_term(t-1) * params.dt; %Update of decay term (ds/dt=-alpha s)
    end
    
    % compute training data as [x; dx; ddx]
    training_data = spline(1:size(demostration(1:params.nbDOF,:), 2), demostration(1:params.nbDOF,:), linspace(1, size(demostration(1:params.nbDOF,:), 2), params.nbData)); % resampling
    training_data = [training_data; gradient(training_data) / params.dt]; % velocity computation	
    training_data = [training_data; gradient(training_data(end - params.nbDOF+1:end, :)) / params.dt]; % acceleration computation
    
    % compute forcing term (f_d)
    xTar = demostration(1:params.nbDOF,end);
    forcing_term = training_data(accId,:) - params.alphaY * (params.bettaY * (repmat(xTar, 1, params.nbData) - training_data(posId, :)) - training_data(velId, :));