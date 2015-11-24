function [closestSol] = search_lookupTable(lookupTraj, Tquery)

    nSol = numel(lookupTraj);
    for k=1:nSol 
        % compute the distance between the query and the previously
        % computed solutions
        dist(k) = sqrt( sum( (lookupTraj{k}.vpGrasp.T(1:3,4)-Tquery(1:3,4)).^2 ));
    end
    
    [val, idx] = min(dist);
    
    closestSol = lookupTraj{idx};
    
    fprintf('Closest entry in lookup table: %g\n\n', idx);

end