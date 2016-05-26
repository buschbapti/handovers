function sys = reps_wrapper_IK(sys, rollOutNoisy)
% sys = reps_wrapper_IK(rollOutNoisy, sys, currentT)
 
    for p = 1:numel(rollOutNoisy)
        Cost(:,p)  = rollOutNoisy{p}{1}.ikSol.cost;
        Theta(:,p) = rollOutNoisy{p}{1}.theta_eps;
    end
    Jfinal = -Cost';

    [out] = updateREPS(Theta, Jfinal);
    
    
    for j=1:sys.nDoF
        
        % By adding at end+1 I can keep the history of values.
        sys.dof(j).wMean(:,end+1)  = out.mu(j);

        % The update of the covariance if fishy because the
        % weights are correlated by REPS but I am sampling each
        % of them independently. I have to think more about
        % this part. 
        sys.dof(j).wCov(:,:,end+1) = out.cov(j,j);

    end

end







