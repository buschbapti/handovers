function [run ] = roll_out_refFrame(run, sys, currentT, i_rollout, rotationCenter, noiseScaling)
% [run ] = roll_out_refFrame(run, sys, currentT, i_rollout, rollOutType, rotationCenter)
%
% Perturb the reference frame location by adding gaussian noise extracted
% where the variance is given in sys.dof(j).wCov(:,:,1).
%
% Use the parameters in sys.dof(1:n).wMean(end) with/out noise and update
% to move the trajectory in currentT to a new reference frame location.
% 
% Note that translations are incremental such that perturbedTranslation = dTranslation.
% Note that rotations are incremental such that perturbedRotation = dTheta.
%
% Therefore, to recover the solution you have to use meanA +dA.
% What REPS will do is to update meanA according to the several {1...K}
% perturbed solutions (meanA+dA)_k
% This procedure is consistent with updating the weight parameters of a
% trajectory encoded as a ProMP, for example.

           
    for j=1:numel(sys.dof) % number of dofs to be perturbed
        eps(j) = sqrt( sys.dof(j).wCov(:,:,1) )*randn(1)*noiseScaling;
    end


    % perturb the reference frame relative to currentT
    % =====================   
    % 1. Translate perturbations   
    perturbedTranslation = [sys.dof(1).wMean(end) + eps(1)
                            sys.dof(2).wMean(end) + eps(2)
                            0];
                        
    if ~sum(perturbedTranslation)==0 % only do this part if parameters are nonzero
        [newT.T, newT.p] = homogTransfTranslate( perturbedTranslation, currentT.T,...
                                                          struct('type', 'relative', 'plot_flag', 0) );
    else
        % If the perturbation vector is zero, simply copy currentT
        newT.T = currentT.T;
        newT.p = squeeze(currentT.T(1:3,4,:));
    end
                                     
    % 2. Relative rotation. Rotations are incremental in relation to the
    %    current angle. 
    perturbedRotation = sys.dof(3).wMean(end) + eps(3);
    
    tmp.plot_flag  = 0;
    tmp.center_of_rotation = rotationCenter + perturbedTranslation;
    if perturbedRotation~=0
        [newT.T, newT.p] =  homogTransfRotate( [0 0 perturbedRotation], newT.T, tmp );
    end
    
    
    if 0  % good to debug!!!
        tmp.hfig = figurewe('fasdf');
        view([-1 -1 0.5]);
        xlabel 'x'; ylabel 'y'; zlabel 'z';
        tmp.axis_length = 1.15; 
        tmp.nMaxPlots = 20;    
        tmp.axis_length = 0.1;
        homogTransfPlot(currentT.T, tmp);
        homogTransfPlot(newT.T, tmp);  
        axis 'tight'
    end        
  
    if isempty(run)
        i_rollout = 1;
    end    

    run{i_rollout}{1}.T     = newT.T;
    run{i_rollout}{1}.p     = newT.p;
    run{i_rollout}{1}.theta = 'roll_out_refFrame.m bug';  %'%currentT.T(1:3,4,1)'; % there may be a bug here!!!
    run{i_rollout}{1}.eps   = eps;
    
    % because translations and rotations are relative, should I consider theta=0??
    run{i_rollout}{1}.theta_eps = [sys.dof(1).wMean(end) + eps(1)
                                   sys.dof(2).wMean(end) + eps(2)
                                   sys.dof(3).wMean(end) + eps(3)
                                   ]; 
 

end





















