function [out] = updateREPS(Theta, Jfinal)
%
% Theta  [n_param, n_rollouts]: the parameters of n_rollouts%
% Jfinal [n_rollouts, 1]:       the final cummutative reward of each
%                               rollout



    % find the weights for computing the weighted sample average
    [d, divKL] = optimize(Jfinal);

    % ...................................................
    out.avgRew = mean(Jfinal);
    out.divKL = divKL;
    out.d = d;

    %Theta = model.w.mean_full(model.w.index{j});
    [mu, sigma] = weightedMLUpdate(d, Theta) ;

    out.cov = sigma; % Is this covariance or standard deviation???
    out.mu = mu;
      
   
end

function [d, divKL] = optimize(J)
    % Optimization problem settings
    options = optimset('GradObj', 'on', ...
        'Display', 'off', ...
        'MaxFunEvals', 300 * 5, ...
        'Algorithm', 'interior-point', ...
        'TolX', 10^-8, ...
        'TolFun', 10^-12, ...
        'MaxIter', 300);
    lowerBound = 1e-8; % eta > 0
    upperBound = 1e8; % eta < inf

    
    N_MAX = numel(J); %<= THIS HAS TO CHANGE WHEN IMPORTANCE SAMPLING IS ON

    eta0 = 1;


    eta = fmincon(@(eta)dual(eta,J, N_MAX), ...
        eta0, [], [], [], [], lowerBound, upperBound, [], options);

    % Perform weighted ML to update the high-level policy
    d = exp( (J - max(J)) / eta );

    % Compute KL divergence
    % gjm: Note that here the KL divergence is between the weights.
    %      qWeighting is always on
    %      pWeighting is the weights to update the new policy
    %      The divergence is computed as a stop criteria.
    qWeighting = ones(N_MAX,1);
    pWeighting = d;
    divKL = getKL(pWeighting, qWeighting);
    
    
end

function [mu, Cov] = weightedMLUpdate(d, Theta)

    % Updating the mean of the parameters
    % Generically this is an operation of computing the mean and
    % covariance on weighted samples
    % .......................................

    % Roll-out weights
    roll_out_weights = d / sum(d);
    mu = Theta * roll_out_weights;

    % Weighted covariance
    Cov = zeros( numel(mu), numel(mu) );
    for k = 1 : size(Theta,2)
        Cov = Cov + (d(k) * (Theta(:,k) - mu) * (Theta(:,k) - mu)');
    end

    % normalize covariance
    Z = (sum(d)^2 - sum(d.^2)) / sum(d);
    Cov = Cov / Z;

    Cov = Cov + 1e-16*eye(size(Cov));
    if 1 % original from Simone
        Cov = nearestSPD(Cov);
    else
        %Cov = regularizeCovariance(Cov); % inputs incomplete!!!
        Cov = 'not_in_use';
    end

    
end

% DUAL FUNCTION
function [g, gd] = dual(eta, J, N_MAX)

    epsilon = 0.9;

    % Numerical trick
    maxJ = max(J);
    J = J - maxJ;

    A = sum(exp(J / eta)) / N_MAX;
    B = sum(exp(J / eta) .* J) / N_MAX;

    g = eta * epsilon + eta * log(A) + maxJ; % dual function
    gd = epsilon + log(A) - B / (eta * A);   % gradient
    
end

% Approximate the Kullback-Leibler KL(q||p) divergence beetween two
% distributions q (the old one) and p (the new one) using the weights for
% a Maximum-Likelihood update.
% If no weights for q are provided, they are assumed to be 1.
function div = getKL(pWeighting, qWeighting)

    if(nargin == 1)
        qWeighting = ones(length(pWeighting));
    end
    qWeighting = qWeighting / sum(qWeighting);
    pWeighting = pWeighting / sum(pWeighting);
    index = pWeighting > 10^-10;
    qWeighting = qWeighting(index);
    pWeighting = pWeighting(index);

    div = sum(pWeighting .* log(pWeighting ./ qWeighting));

end

