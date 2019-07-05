function [H, X, weights, F] = fitDMP(forcing_term, decay_term, params)
    % compute activation of each RBF
    H = zeros(params.nbRBF, params.nbData);
    
    for i=1:params.nbRBF
        H(i,:) = gaussPDF(decay_term, params.Mu(:,i), params.Sigma(:,:,i));
    end
    H = H ./ repmat(sum(H),params.nbRBF,1); % normalisation, i.e. all RBF at any time add up to 1
    
    % fit DMP
    X = [];
    for d=0:params.polDeg 
        X = [X, decay_term.^d'];
    end
    
    Y = forcing_term';
    for i = 1:params.nbRBF
        W = diag(H(i, :));
        %weights(:,i) = (X' * W * Y) / (X' * W * X); % only valid for 0-order polynomial
        weights(:,:,i) = (X' * W * X) \ (X' * W * Y); % weighted least squares
    end
    
    retrF = zeros(params.nbData, params.nbDOF);
    for t=1:params.nbData
        for i=1:params.nbRBF
            retrF(t,:) = retrF(t,:) + H(i,t) * X(t,:) * weights(:,:,i);
        end
    end
    F = retrF';