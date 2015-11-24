function [weight, basisGn] = weight_space_mapping_viaLinRegression(yIn, nBasis)

    nDemo = numel(yIn(:,1));
    nTraj = numel(yIn(1,:)); % length of each demonstration

    mu_location = linspace(0,1,nBasis); 

    dt     = 1/nTraj;    % such that z = [dt 2dt 3dt. . . 1], where [1 x nTraj]
    phase  = Phase(dt);  % replaces the phase

    % create empty object
    % weight = Weight(nBasis, 1, nTraj, nDemo);


    % this is noise to be added in the matrix diagonal such that
    % inversion is stable
    linRegRidgeFactor = 1e-8*eye(size(mu_location,2), size(mu_location,2) );   

    sigma = 0.05*ones(1,nBasis); % kernel width
    [basis.Gn, basis.Gndot, basis.Gnddot] = generateGaussianBasis( phase, mu_location, sigma);

    weight = least_square_on_weights(basis.Gn, yIn, linRegRidgeFactor);
    basisGn = basis.Gn;
    
end 


function wOut = least_square_on_weights(Gn, demoq, linRegRidgeFactor)
% Maps the trajectories to weights

    dbg       = 0;

    nDemo    = numel(demoq(:,1));
    nJoints  = 1;
    nBasis   = numel(Gn(1,:));

    % find weights by normal solution with Moore-Penrose Pseudo-Inverse
    MPPI = (Gn'*Gn  + linRegRidgeFactor) \ Gn';      

    % First get the weights of each trajectory for each joint
    for k = 1:nDemo
        w_   = MPPI*demoq(k,:)';
        wOut(k,:) = w_';
        if dbg % reproduce each trajectory
            figurew(['learned weights ', num2str(k) , 'joint', num2str(j)]);
            title 'Comparing trajectory reproduction by weights'
            plot(demoq(k,:), 'bo-');
            plot(w_'*Gn', 'r-');
        end
    end

end % learn weights



