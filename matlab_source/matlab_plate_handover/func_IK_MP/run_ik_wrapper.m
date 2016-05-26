function [currentT, param] = run_ik_wrapper(robot, currentT, param, i, type) 


    % Use vrep for IK
    % =====================================
    [ik_hist, param]                    = ikine_vrep( robot, currentT.T, 0, param);
    [ik_hist.cost, ik_hist.neResampled] = IK_cost(robot, ik_hist, param);    
 
    % ======================================
    % Decide here how to plot the ik solution    
    switch type
        case 'noiseless'
            try % clean figure
                delete(param.plot.frame_currentSol.handleCurves);
                param.plot.frame_currentSol.handleCurves =[];
            end
            % plot ik input of the current noiseless solution
            if ( mod(i, 3)==0 ) || ( i==1 ) || ( i==param.reps.nUpdates )
                if ~isempty(param.hfig.hworkspc)
                    figure(param.hfig.hworkspc);
                    plot3(currentT.p(1,:), currentT.p(2,:), currentT.p(3,:), sty('b', 'none', 1, '-')   );
                    plot3(currentT.p(1,:), currentT.p(2,:), -0.475*ones(1, numel(currentT.p(2,:))), sty([0.6 0.6 0.6], 'none', 2, '-')   );
                end
            end
            % plot ik output
            color_now  = [param.plot.colorGrad(i) param.plot.colorGrad(i) param.plot.colorGrad(i)];
            [param] = plot_noiseless_solution(robot, ik_hist, param, color_now  );
        case 'exploration'
            [param] = plot_exploration(ik_hist, param);
            
            if ~isempty(param.hfig.frame_exploration)
                figure(param.hfig.frame_exploration);
                param.hfig.frame_exploration(end+1) = ...
                     plot3(currentT.p(1,:), currentT.p(2,:),  currentT.p(3,:), sty( [0.75 0.75 1], '.', 3, '--')   );            
            end
    end

    currentT.ikSol = ik_hist;    
end

function [param] = plot_noiseless_solution(robot, ik_hist, param, color_now )
    
    % plot error of inverse kinematics
    if ~isempty(param.hfig.ikine_error)
        figure(param.hfig.ikine_error);
        plot(ik_hist.neResampled, sty(color_now, [], 2,'-',5) );
    end

    if param.plot.frame_currentSol.plotflag   % plot current solution
        [~, hTmp] =  homogTransfPlot(ik_hist.T, param.plot.frame_currentSol);
        param.plot.frame_currentSol.handleCurves = hTmp;
        
        for kk=1:3
            param.plot.frame_leaveTrace.axesPlotStyle{kk}.Color = color_now;
        end
    
        % plot the current solution in gray so as to leave a trace
        homogTransfPlot(ik_hist.T, param.plot.frame_leaveTrace);
    end

    if param.plot.animateEveryIteration
        h.del = []; 
        nTraj = numel(ik_hist.q(:,1));
        for k=1:nTraj
            [T_, p_ ] = FK_and_homog_transf( robot.L, robot.alpha, robot.d, ik_hist.q(k,:) );
            h.del = [h.del  plot(p_(:,  1), p_(:,  2), styw('k', 'o', 4,' -',10 )  )];
            title(['Step ' num2str(k) ' of '  num2str(nTraj)]);       
            drawnow;
            pause(0.2) %keyboard
            if  1% k ~= numel(traj.q_ik(:,1))
                delete(h.del); h.del=[];
            end
        end
    end
    
end

function [param] = plot_exploration(ik_hist, param)

    color_now = 'r';
    
    % plot error of inverse kinematics
    if param.hfig.ikine_error
        figure(param.hfig.ikine_error);
        hTmp1 = plot(ik_hist.neResampled, sty(color_now, [], 1,'-', 5) );
        param.plot.frame_exploration.handleCurves = ...
                                           [param.plot.frame_exploration.handleCurves hTmp1];  
    end
    % plot current solution
    % .....................................
    if ~isempty(param.hfig.frame_exploration)
        figure(param.plot.frame_exploration.hfig);
        pos = getPositionFromT(ik_hist.T); % seems to be the current position obtained by IK
        hTmp2 = plot3( pos(1,:), pos(2,:), pos(3,:) , param.plot.frame_exploration.pathPlotStyle);  
        param.plot.frame_exploration.handleCurves = ...
                                           [param.plot.frame_exploration.handleCurves hTmp2];        
    end

   

end




