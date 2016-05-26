function [sys, currentT, finishFlag] = optimization_loop(sys, robot, initT, param)
    

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
        ik_satisfied = plot_and_assess_IK_solution(param.plot_handles.costFrame, run_eval, i, param.ikine.minCost);

        try % Delete the previous traces of exploration
            delete(param.plot.frame_exploration.handleCurves);
            param.plot.frame_exploration.handleCurves=[];
        end  

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
            costDeviationFromMean = compute_cost_deviation_mean(run_eval{1}{2}, robot.initialCartesianTrajectory.p, param);
            
            % Second, update the trajectory to the current ref frame solution
            [plotT, run_eval{1}{2}, param] = update_ProMP_traj_with_IK_frame( run_eval{1}{2}, sys{1},...
                                           param, robot.opt.plotShadowzheight, ZERO_NOISE );

            % Plot the current solution
            % ======================================
            try delete(hworkspaceTemp); end
            hworkspaceTemp = plot_current_solution(plotT.T, param.hfig);           
            drawnow;
            
            try; delete(hrr); end 
            hrr = [];           
    
            [run_eval{1}{2}, currentViaPointError, finishFlag]= getCostCollaboration(run_eval{1}{2}, param, costDeviationFromMean); % compute cost
            
            % plot
            plot_MP_solution(param.plot_handles.costFrame, run_eval, currentViaPointError, i);          

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




function [hworkspaceTemp] = plot_current_solution(T, hfig)

 %   sys = getPreviousSol.obj2.sys;
 %   param = getPreviousSol.obj2.param;
 %   robotOriginal = getPreviousSol.obj2.initT;
    
%     
%     % Get trajectory back at original frame position
%     % =============================================
%     Gn = sys{2}.basis.Gn;
%     for j = 1:sys{2}.nDoF, % for each dmp independently
%         p(:,j) = Gn*sys{2}.dof(j).wMean(:,end);
%     end    
% 
%     % Move the MP trajectory to the current IK solution frame
%     % ===================================================
%     nTraj = numel(p(:,1));
%     
%     % 0. Create the tranformation matrix while using the original end effector orientation
%     for t =1:nTraj
%         oldMP.T(:,:,t) = [eye(3,3) [p(t,1); p(t,2); p(t,3)] ;[0 0 0 1]] ;
%     end
%  
%     runIK = roll_out_refFrame([], sys{1}, oldMP, [], param.frameRotateCenter, 0);
%    
%     
%     % Maintain the original orientation of all points
%     % =====================================================
%     for t=1:nTraj
%         runIK{1}{1}.T(1:3,1:3,t) = robotOriginal(1:3,1:3,t); 
%     end
%     finalSol = runIK{1}{1};
    
    param2.hfig = hfig;
    param2.axis_length = 0.05; % param.axis_length = 0.050;
    param2.nMaxPlots = 5;
    param2.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 0 0]);
    param2.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [.0 1 .0]);
    param2.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [.0 .0 1]);
    param2.shadowHeight     = -1.22; % height of the floor
    param2.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'b', 'Marker', 'none');    
    [~, hworkspaceTemp] = homogTransfPlot(T, param2);
            
end
























