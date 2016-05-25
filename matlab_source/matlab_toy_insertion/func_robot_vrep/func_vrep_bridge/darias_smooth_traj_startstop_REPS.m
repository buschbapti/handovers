function [] =  darias_smooth_traj_startstop_REPS(vrepTraj, param)


    nRollOut = param.nRollOut;


    % Optimize start
    % =====================
    % search for the joint with the highest initial velocity    
    [val, dof] =  max( abs(vrepTraj.q(1,:)) );
    y = vrepTraj.q(:,dof);
    t = 0:vrepTraj.dt:(numel(y)-1)*vrepTraj.dt;

    nResampleTraj = 200; %
    [tOrigResampled, yOrigResampled] = resample(t, y, nResampleTraj);

    diff_yOrigResampled = diff(yOrigResampled);

    % ===============================================
     dtOriginal = diff(tOrigResampled);
    [weight, basisGn] = weight_space_mapping_viaLinRegression(dtOriginal, param.nBasis);

    if 0
        figurew('checkEncoding_dt'); ylabel 'dt';
        plot(dtOriginal, 'b.-');
        plot(weight*basisGn', 'r.-');

        figurew('checkEncoding_t'); ylabel 'dt';
        plot([0 cumsum(dtOriginal)], 'b.-');
        plot([0 cumsum(weight*basisGn')], 'r.-');
    end

    [run_eval, run_noisy] = createLoopDataHolders( param.nRollOut);


    h.cost = figurew('Cost');
    xaxis([1 param.nUpdates]);

    h.qdot = figurew('Velocity');

    repsPolicy.wMean = weight';
    repsPolicy.wCov  = covarianceStomp( param.nBasis, [10  10] );
    repsPolicy.Gn = basisGn;

    repsPolicy.wCov  = 1e-9*repsPolicy.wCov;

    for i = 1:param.nUpdates

        % Run clean roll-out

        noise_mult=0;
        run_eval{1}      = roll_out(run_eval{1},repsPolicy, noise_mult, param);
        yd = diff_yOrigResampled./diff(run_eval{1}.t);
        run_eval{1}.costTraj = cost_traj(tOrigResampled, run_eval{1}.t, yd, param);
        run_eval{1}.sumCost = sum(run_eval{1}.costTraj);

        figure(h.cost);
        plot(i, run_eval{1}.sumCost, styw_nl('b', 'o', [],[],10));

        try; delete(h.temp); end
        figure(h.qdot);
        h.temp = plot(run_eval{1}.t(1:end-1), yd, sty('b', [], 5) );
        drawnow;

        if run_eval{1}.costTraj < param.targetCost
           return 
        end

        for p = 1:nRollOut
            fprintf('  > Noisy IK. %g of %g\n', p, nRollOut);
            noise_mult = 1;
            run_noisy{p}   = roll_out(run_noisy{p}, repsPolicy, noise_mult, param);
            yd = diff_yOrigResampled./diff(run_noisy{p}.t);
            run_noisy{p}.costTraj = cost_traj(tOrigResampled, run_noisy{p}.t, yd, param);
            run_noisy{p}.sumCost = sum(run_noisy{p}.costTraj);
        end       

       % update the policy
        repsPolicy = reps_wrapper(repsPolicy, run_noisy);

    end % nUpdates


    figure(h.qdot);
    yd_Orig = diff_yOrigResampled./diff(tOrigResampled);
    plot(tOrigResampled(1:end-1), yd_Orig, sty('r', [], 3) );


        
    
end

function policy = reps_wrapper(policy, rollOutNoisy)
% sys = reps_wrapper_IK(rollOutNoisy, sys, currentT)
 
    for p = 1:numel(rollOutNoisy)
        Cost(:,p)  = rollOutNoisy{p}.sumCost; % single scalar value
        Theta(:,p) = rollOutNoisy{p}.theta_eps'; % vector nBasis
    end
    Jfinal = -Cost';

    [out] = updateREPS(Theta, Jfinal);
    
    
    for j=1
        
        % By adding at end+1 I can keep the history of values.
        policy.wMean(:,end+1)  = out.mu;

        % The update of the covariance if fishy because the
        % weights are correlated by REPS but I am sampling each
        % of them independently. I have to think more about
        % this part. 
        policy.wCov(:,:,end+1) = out.cov;

    end

end



function Cost = cost_traj(tOrig, tNow, yd, param)

    Qinitial = param.Qinitial;
    Qfinal   = param.Qfinal;
    QfinalTimeRate = param.QfinalTimeRate; % avoids stretching time too much    
    QmaxVelAllowed = param.QmaxVelAllowed;
    QfullTraj = param.QfullTraj;
    
    Cost = zeros(size(yd));
    
    % Cost start and end
    iInit = param.Qinitial_idx;
    Cost(1:iInit)   = Cost(1:iInit)   + Qinitial*yd(1:iInit).^2;
    
    iFinal = param.Qfinal_idx;
    Cost(end-iFinal:end) = Cost(end-iFinal:end) + Qfinal*yd(end-iFinal:end).^2;
    
    % find indices where velocity exceeds some maximum desired value
    idx = abs(yd) >= param.maxVel;
    Cost(idx) = Cost(idx) + QmaxVelAllowed*yd(idx).^2;
    
    % general trajectory cost
    Cost = Cost + QfullTraj*yd.^2;
    
    % punish for making the difference of time too large but does not
    % punish if trajectory gets shorter
    if tNow(end) > tOrig(end) 
        CostExtra = QfinalTimeRate*(tOrig(end)-tNow(end)).^2;
    else
        CostExtra = 0;
    end

    Cost = [Cost CostExtra];
    

end

function run = roll_out(run, policy, noise_mult, param)

    % Select which type of covariance to use
    if param.decreaseCovarianceAsPi2
        wCov = policy.wCov(:,:,1); % use always the initial covariance.
    else
        wCov = policy.wCov(:,:,end); % use the lates covariance updated by REPS
    end

    % perturb the weight vector
    epsilon = mvnrnd(zeros(1, numel(policy.wMean(:,end))), noise_mult.*wCov);

    epsilon = abs(epsilon);
    
    Gn = policy.Gn;

    new_dT   = Gn*policy.wMean(:,end)    + Gn*epsilon';
    
    
    run.t   = [0; cumsum(new_dT)];
    %run.epsilon   = epsilon;
    run.t = run.t';
    run.theta_eps = policy.wMean(:,end)' + epsilon;
    
    
end



function [tNew, yNew] = resample(t, y, nTraj)

    tNew = linspace(0, t(end), nTraj);
    yNew = interp1(t, y, tNew);

    dbg=0;
    if dbg
        figurew('ty');
        plot(t, y, 'bo-'); plot(tNew, yNew, 'r.-');
    end

end




function [run_eval, run_noisy] = createLoopDataHolders( nRollOut)

    % Creating initial "run" structure
    run_eval{1} = createMainStruc( );
    
    run_noisy{nRollOut,:}=[];
    for nr = 1:nRollOut
        run_noisy{nr,:} = run_eval{1};
    end    
    
end

function run = createMainStruc( )
   
    run.y     = [];
    run.theta = [];
    run.theta_eps = [];

end





