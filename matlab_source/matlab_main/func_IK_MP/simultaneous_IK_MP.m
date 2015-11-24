function [sys, param] = simultaneous_IK_MP(robot, run_noisy, sys, param)

    dbg = 0;
    NOISE_ON_IK_FRAME = 1;
    
    for i_rollout = 1:param.reps.nRollOut
        param.looptic = tic;
        
        try delete(hb); end  
        hb = [];           
        
        % MP ==============================================
        % perturb the parameters of the trajectory in relation to the original
        % reference frame
        i_update_fake =1 ;
        noiseMP = getNoiseAsPi2(i_update_fake,  param.reps.nUpdates, param.reps.minNoiseMultiplierValue);
        run_noisy{i_rollout}{2} = roll_out_trajectory(sys{2}, run_noisy{i_rollout}{2}, param, noiseMP);
        costDeviationFromMean   = compute_cost_deviation_mean(run_noisy{i_rollout}{2}, robot.initialCartesianTrajectory.p, param);
         
        if dbg
            % plot the current MP solution in the original frame
            hb = [hb  plot_ref_frame(param.hfig.currentIKMPsol, run_noisy{i_rollout}{2} , 'MP', hb)];
        end
             
        % move the perturbed trajectory on a newly perturbed reference frame
        % =============================================        
        [run_noisy{i_rollout}{1}, run_noisy{i_rollout}{2}, param] = ...
                      update_ProMP_traj_with_IK_frame(run_noisy{i_rollout}{2}, sys{1}, param, ...
                                                      robot.opt.plotShadowzheight, NOISE_ON_IK_FRAME );
        % do IK with the new reference frame
        % =============================================
        tmp = param.hfig.single_IK_iteration;
        param.hfig.single_IK_iteration = [];
       
        run_noisy{i_rollout}{1} = resample_for_fast_IK(sys{1}.resampleIdx, run_noisy{i_rollout}{1});
        
        
        [run_noisy{i_rollout}{1}, param] = run_ik_wrapper(robot, run_noisy{i_rollout}{1}, param, [],  'exploration');
        param.hfig.single_IK_iteration = tmp;
        
        % compute the cost
        run_noisy{i_rollout}{2} = getCostCollaboration(run_noisy{i_rollout}{2}, param, costDeviationFromMean); % compute cost
        fprintf('%g sec. > Noisy IK+MP. %g of %g\n', toc(param.looptic), i_rollout, param.reps.nRollOut);
    end
    
    try delete(hb); end     

    sys = reps_wrapper_IK_MP(sys, run_noisy, param.reps.nRollOut, param);
    
end






















