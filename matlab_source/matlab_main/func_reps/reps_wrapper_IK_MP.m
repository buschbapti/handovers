function sys = reps_wrapper_IK_MP(sys, run_noisy, nRollOut, param)
% sys = reps_wrapper(run_noisy, sys, currentT)
   
    
    %% IK 
    % Cost and IK parameters
    for p = 1:nRollOut
        Cost_IK(:,p)  = run_noisy{p}{1}.ikSol.cost;
        Theta_IK(:,p) = run_noisy{p}{1}.theta_eps;
    end
    Jfinal_IK = -Cost_IK';
    
    %% Motion Planning
    
    ThetaMPcombined= [];
    for j=1:run_noisy{1}{2}.n_dof

        for p = 1:nRollOut
            R_motionPlanning{j}(:,p)     = -run_noisy{p}{2}.dof(j).costHistory;
            Theta_MP{j}(:,p) =  run_noisy{p}{2}.dof(j).theta_eps';
        end
        J{j} = cumsum(R_motionPlanning{j});
        Jfinal_MP(:,j) = J{j}(end,:)';  

        ThetaMPcombined = [ ThetaMPcombined; Theta_MP{j}];
    end

    Jfinal_MP_combined = sum(Jfinal_MP,2);
    
    %% Update IK and MP at the same time
    
    Theta  = [ThetaMPcombined; Theta_IK];
    
    % TOD : NORMALIZE REWARDS HERE????
    Weight_IK_MP = param.reps.Weight_IK_MP;
    
    if 0 % try to normalize the cost here
        Jfinal_IK = Jfinal_IK./sum(Jfinal_IK);
        Jfinal_MP_combined = Jfinal_MP_combined./sum(Jfinal_MP_combined);
    end
    Jfinal = Weight_IK_MP(1)*Jfinal_IK + ...
             Weight_IK_MP(2)*Jfinal_MP_combined;
         
    out = updateREPS(Theta, Jfinal);
    
    %% Update sys for motion planning
    nTheta_MP = numel( run_noisy{1}{2}.dof(j).theta_eps );
    for j = 1:run_noisy{1}{2}.n_dof

        irangeJoint = (1+(j-1)*nTheta_MP):j*nTheta_MP;
        % keep weights at extremities constant

        if 0%param.reps.anchorInitial_nWeights
            idx = 1:param.reps.anchorInitial_nWeights;
            idxMu = irangeJoint(idx);
            out.mu( idxMu ) = sys{2}.dof(j).wMean(idx,1);
        end

        if 0%param.reps.anchorFinal_nWeights
            idx = ( numel(irangeJoint)-param.reps.anchorFinal_nWeights ):numel(irangeJoint);
            idxMu = irangeJoint(idx);
            out.mu( idxMu ) = sys{2}.dof(j).wMean(idx,1);
        end

        sys{2}.dof(j).wMean(:,end+1)   = out.mu(irangeJoint);
        sys{2}.dof(j).wCov(:,:,end+1)  = out.cov(irangeJoint,irangeJoint);
    end
    
    %% Update sys for IK
    for j = 1:sys{1}.nDoF        
        ir = numel( out.mu) -sys{1}.nDoF+(j);
        sys{1}.dof(j).wMean(:,end+1)  = out.mu(ir);
        % The update of the covariance if fishy because the
        % weights are correlated by REPS but I am sampling each
        % of them independently. I have to think more about
        % this part. 
        sys{1}.dof(j).wCov(:,:,end+1) = out.cov(ir,ir);        
    end
    
    
end
















