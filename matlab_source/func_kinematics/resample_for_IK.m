function [init, idx] = resample_for_IK(init, param)
% function init = resample_for_IK(init, param)
% This is an auxiliar function that is used to downsample the IK trajectory
% This does not affect the number of samples of the MP part.

    % downsample for ik
    irold = 1:numel(init.p(1,:));
    told  = linspace(0,1,numel(init.p(1,:)));
    tnew  = linspace(0,1,param.ikine.createInitTraj.nSamples);
    irnew = interp1(told, irold, tnew); % indexes for a resampled trajectory
                                        % (but at this stage they are not
                                        % integers)
    idx = floor(irnew);

    init.p = init.p(:,idx); 
    %init.pdot = init.pdot(:,init.idx); 
    init.T = init.T(:,:,idx);
    
end