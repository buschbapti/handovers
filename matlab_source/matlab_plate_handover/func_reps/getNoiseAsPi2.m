function noise_mult = getNoiseAsPi2(i, nUpdates, minNoiseScale)
% function noise_mult = getNoiseAsPi2(i, nUpdates, minNoiseScale)
%
% This function computes the scaling factor for the noise variance as it is done in Pi2 [see Schaall
% code in his website].
% This is crearly a heuristic that is not given much attention. REPS will, in fact, decrease the
% exploration automatically, but this method gives more control to the user.
%
%   INPUT
%       i: current update number
%       nUpdates: total number of updates
%       minNoiseScale: the minimum value to which scale the noise variance. If you set to zero, that
%                      means that the noise will decay to zero at the last update.
%

    %nUpdates = nUpdates-0;
    % run learning roll-outs with a noise annealing multiplier
    noise_mult = double(nUpdates - i)/double(nUpdates);
    noise_mult = max([minNoiseScale   noise_mult]);
    
    
end