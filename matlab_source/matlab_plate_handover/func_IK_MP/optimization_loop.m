function [sys, currentT, finishFlag] = optimization_loop(sys, robot, initT, param)
    
    haux.h = param.hfig.auxDeviation;

    ZERO_NOISE = 0;
    
    nRollOut = param.reps.nRollOut;
    [run_eval, run_noisy] = createLoopDataHolders(sys, nRollOut);

    % this is the initial trajectory guess of the robot side only
    % The trajectory already has the correct orientation by construction.
    currentT  = initT;
    
    
    for i = 1:param.reps.nUpdates            
        
        % noiseless rollout to evaluate current cost of current solution
        % ============================================        
        % compute the IK solution 
        run_eval                = roll_out_refFrame(run_eval, sys{1}, currentT, 1, param.frameRotateCenter, ZERO_NOISE);
        
        [run_eval{1}{1}, param] = run_ik_wrapper(robot, run_eval{1}{1}, param, i,  'noiseless');
        ik_satisfied = plot_and_assess_IK_solution(param.hfig.costFrame, run_eval, i, param.ikine.minCost);

        try % Delete the previous traces of exploration
            delete(param.plot.frame_exploration.handleCurves);
            param.plot.frame_exploration.handleCurves=[];
        end  

        try delete(hworkspaceTemp); end 
        try delete(hworkspaceTemp1); end
        hworkspaceTemp1 = plot_current_solution(run_eval{1}{1}.T, param.hfig.hworkspc, robot.opt.floorHeight, 'IK_ref');  
        hworkspaceTemp1 = [hworkspaceTemp1  plot_current_solution(run_eval{1}{1}.ikSol.T, param.hfig.hworkspc, robot.opt.floorHeight, 'IK_sol')];
        title('IK solution only');
        drawnow;       
        if ~ik_satisfied % IK still not satisfied so only do IK updates, and not MP. 
            
            for p = 1:nRollOut
                param.looptic = tic;
                run_noisy     = roll_out_refFrame(run_noisy, sys{1}, currentT,  p, param.frameRotateCenter, 1);
                [run_noisy{p}{1}, param] = run_ik_wrapper(robot, run_noisy{p}{1}, param, [],  'exploration');
                fprintf('%g sec. > Noisy IK. %g of %g\n', toc(param.looptic), p, nRollOut);
            end       

            % updates the location of the reference frame.
            sys{1} = reps_wrapper_IK(sys{1}, run_noisy);          

            
            drawnow;
            
        else  % assess if MP is satisfied
            
            % Clean roll-out on noiseless trajectory
            % ==============================================
            run_eval{1}{2} = roll_out_trajectory(sys{2}, run_eval{1}{2}, param, ZERO_NOISE);
            [costDeviationFromMean, haux] = compute_cost_deviation_mean(run_eval{1}{2}, robot.initialCartesianTrajectory.p, param, haux);
            
            % Second, update the trajectory to the current ref frame solution
            [plotT, run_eval{1}{2}, param] = update_ProMP_traj_with_IK_frame( run_eval{1}{2}, sys{1},...
                                             param, robot.opt.plotShadowzheight, ZERO_NOISE );
                                         
            if 0
                        % do IK with the new reference frame
                        % =============================================
                        tmp = param.plot_handles.single_IK_iteration;
                        param.plot_handles.single_IK_iteration = [];

                        plotTr = resample_for_fast_IK(sys{1}.resampleIdx, plotT);

                        [run_noisy{i_rollout}{1}, param] = run_ik_wrapper(robot, plotTr, param, [],  'exploration');
                        param.plot_handles.single_IK_iteration = tmp;

                        % compute the cost
                        run_noisy{i_rollout}{2} = getCostCollaboration(run_noisy{i_rollout}{2}, param, costDeviationFromMean); % compute cost
                        fprintf('%g sec. > Noisy IK+MP. %g of %g\n', toc(param.looptic), i_rollout, param.reps.nRollOut);


                        % Plot the current solution
                        % ======================================
                        try delete(hworkspaceTemp); end 
                        try delete(hworkspaceTemp1); end
                        hworkspaceTemp = plot_current_solution(plotT.T, param.hfig.hworkspc, robot.opt.floorHeight, 'IKMP_ref');
                        hworkspaceTemp = [hworkspaceTemp plot3(plotT.T(1,4,param.mp.viaPoint.indexes(2)), plotT.T(2,4,param.mp.viaPoint.indexes(2)), ...
                                                               plotT.T(3,4,param.mp.viaPoint.indexes(2)), styw('r', 'o', 1, [], 15)  )];
            %%%%%%%%%%%%%%%%%%%%%%%            %hworkspaceTemp = plot_current_solution(plotT.T, param.hfig.hworkspc, robot.opt.floorHeight, 'IKMP_sol');
                        drawnow;
            end
            
            try; delete(hrr); end 
            hrr = [];           
    
            [run_eval{1}{2}, currentViaPointError, finishFlag]= getCostCollaboration(run_eval{1}{2}, param, costDeviationFromMean); % compute cost
            
            % plot
            plot_MP_solution(param.hfig.costFrame, run_eval, currentViaPointError, i);          

            if finishFlag
                break
            end

            % main simultaneous roll-outs
            % ============================================
            fprintf('Simul IK MP\n');      
            [sys, param] = simultaneous_IK_MP(robot, run_noisy, sys, param);            
            
            % update the new reference trajectory as the one given in sys{2}
            currentT = updateShapeOfReferenceTrajectory(sys{1}.resampleIdx, sys{2}, currentT);
            
            drawnow;
            
        end        
        fprintf('\nFinished update %g of %g.\n', i, param.reps.nUpdates);        
    end

        
    try; delete(hrr); end
    
    % clean up the figures from the last noisy exploration
    try % Delete the previous traces of exploration
        delete(param.plot.frame_exploration.handleCurves);
        param.plot.frame_exploration.handleCurves=[];
    end  
    

end

function newT = updateShapeOfReferenceTrajectory(idx, sys, currentT)

    % make equations clean
    Gn     = sys.basis.Gn;       
    for j = 1:sys.nDoF, % for each dmp independently
        p(:,j) = Gn*sys.dof(j).wMean(:,end);
    end
    
    % maintain the same rotation of each state.
    counter = 1;
    for t = idx        
        % preserve orientation of the original trajectory
        R_old = currentT.T(1:3,1:3,counter);        
        newT.T(:,:,counter) = [  [R_old; 0 0 0]  [ p(t,:)'  ; 1]   ];
        newT.p(:,counter)   = p(t,:)' ;
        counter             = counter+1;        
    end
        
end

function run = createMainStruc( nDoF_traj)        
    i=1;
    run{i}.name = 'ref_frame_location_rollouts';
    run{i}.T     = [];
    run{i}.p     = [];
    run{i}.theta = [];
    run{i}.theta_eps = [];
    run{i}.ikSol = [];

    
    i=2;
    run{i}.name  = 'motion_planning_rollouts';
    run{i}.n_dof = nDoF_traj;
    for j=1:nDoF_traj
        run{i}.dof(j).q       = [];
        run{i}.dof(j).qdot    = [];
        run{i}.dof(j).qddot   = [];
        run{i}.dof(j).epsilon = []; % the disturbance
        run{i}.dof(j).theta   = []; % the parameter
        run{i}.dof(j).theta_eps   = []; % the parameter disturbed
    end 
end


function [run_eval, run_noisy] = createLoopDataHolders(sys, nRollOut)

    % Creating initial "run" structure
    run_eval{1} = createMainStruc(sys{2}.nDoF );
    
    run_noisy{nRollOut,:}=[];
    for nr = 1:nRollOut
        run_noisy{nr,:} = run_eval{1};
    end   
    
end




function [hworkspaceTemp] = plot_current_solution(T, hfig, floorH,  type)
    
    switch type
        case 'IK_ref'
            %param2.pathPlotStyle    = struct('LineWidth', 3, 'Color', 'g', 'Marker', 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 12); 
            param2.pathPlotStyle    = styw('g', 'o', 1.5, [], 12); 
        case 'IK_sol'
            %param2.pathPlotStyle    = struct('LineWidth', 3, 'Color', 'r', 'Marker', 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
            param2.pathPlotStyle    = styw('b', 'o', 1.5, [], 7); 
        case 'IKMP_ref'
            param2.pathPlotStyle    = struct('LineWidth', 4, 'Color', 'g', 'Marker', 'none'); 
        case 'IKMP_sol'
            param2.pathPlotStyle    = struct('LineWidth', 3, 'Color', 'b', 'LineStyle', '-');
        otherwise
            error('sdf');
    end
    
    param2.hfig = hfig;
    param2.axis_length = 0.029; % param.axis_length = 0.050;
    param2.nMaxPlots = 50;
    param2.axesPlotStyle{1} = struct('LineWidth', 1, 'Color', [1 0 0]);
    param2.axesPlotStyle{2} = struct('LineWidth', 1, 'Color', [.0 1 .0]);
    param2.axesPlotStyle{3} = struct('LineWidth', 1, 'Color', [.0 .0 1]);
    param2.shadowHeight     = floorH; % height of the floor
  
    [~, hworkspaceTemp] = homogTransfPlot(T, param2);
            
end
























