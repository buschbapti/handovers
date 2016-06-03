function [closestGuess ] = find_closest_guess(xyz, sol)

    dist = [];
    for i = 1:numel(sol)
        dist = [dist  sqrt( sum(sol{i}{1}.T(1:3,4,end)-xyz').^2 )];
    end
    
    [~, idx] = min(dist);
    
    closestGuess = sol{idx};
       
    
end