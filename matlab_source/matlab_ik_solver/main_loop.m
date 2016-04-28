function [DMP, T, q] = main_loop(robot, DMP, h, nUpdates, nRollOut, param, flagReplayOnly)

%     try close(hFig); end
%     hFig = figurew('stopButton');
%     set(hFig,'KeyPressFcn','myKeyPressFcn'); set_fig_position([0.151 0.369 0.0922 0.0833]);
        
    i=1;
    wIK = param.costWeight.IK;
    if flagReplayOnly == 1
        %robot.nTraj= 50;
        nUpdates=1;
        nRollOut=1;
    end    
            
    while i <= nUpdates
        tic

        DMP.getCovGain(nUpdates, i);


        try 
            delete(h.plotCart);
        end
        try 
            delete(h.plotCartIK);
        end
        h.plotCart = [];         h.plotCartIK = [];

        for j = 1:nRollOut
            tic
            if j==nRollOut                
                DMP.theta_mean_pert_Frame(j,:) = DMP.theta_mean_Frame;
                DMP.theta_mean_pert_dmpx(j,:)  = DMP.theta_mean_dmpx;
                DMP.theta_mean_pert_dmpy(j,:)  = DMP.theta_mean_dmpy;
                DMP.theta_mean_pert_dmpz(j,:)  = DMP.theta_mean_dmpz;                
            else               
                DMP.theta_mean_pert_Frame(j,:) = DMP.theta_mean_Frame  + DMP.sampleNoiseFrame();
                DMP.theta_mean_pert_dmpx(j,:)  = DMP.theta_mean_dmpx   + DMP.sampleNoiseDMP('x');
                DMP.theta_mean_pert_dmpy(j,:)  = DMP.theta_mean_dmpy   + DMP.sampleNoiseDMP('y');
                DMP.theta_mean_pert_dmpz(j,:)  = DMP.theta_mean_dmpz   + DMP.sampleNoiseDMP('z');        
            end

            % generate trajectory
            h = DMP.generate_trajectory(h, j, (j-nRollOut), param.plotRollOuts, param.fixViaPointPoses, param.allowDMPShapeChange);
            
            
            DMP.TrollOut(:,:,:,j);
            
            
            if wIK~=0                
                [ikerrorTraj, h, T, q] = robot.IKcost(j, (j-nRollOut), h, DMP.TrollOut(:,:,:,j));
            else
                robot.costIK(j)=0; ikerrorTraj=0; robot.costIK_clean=0;
            end
            DMP.cost(j, (j-nRollOut), [param.costWeight.similarity ...
                     param.costWeight.obstacle param.costWeight.startGoal...
                     param.costWeight.viaPointMid  param.costWeight.odometry...
                     param.costWeight.objectXdist]);        
            toc
        end
        disp(' ');
                
        if ~flagReplayOnly% add the IK cost here
            DMP.cost_total = DMP.cost_total + wIK*robot.costIK;
            %DMP.cost_total = 0.02*randn(size(DMP.cost_total))+DMP.cost_total;
            DMP.updateCME( ); 
        end

        
        % plot stuff
        figure(h.fig); title(['Iter ' num2str(i)]);
        if param.costWeight.viaPointMid
            Tvp = DMP.TrollOut(:,:,DMP.vp.mid.idx,end);
            [~, h_] = homogTransfPlot(Tvp, struct('hfig', h.fig, 'axis_length', 0.3));
            h.plotCart =  [h.plotCart  h_];
        end
        
        figure(h.figcost); 
        plot(i, DMP.cost_total(end), styw('k', 'o', [], 'none', 15));
        plot(i, DMP.cost_viaPoint(end),   sty('b', 'o', [], 'none', 7));
        plot(i, DMP.cost_startEnd(end),   sty('b', 'x', 1, 'none', 50));
        plot(i, DMP.cost_similarity(end), sty('r', 'o', [], 'none', 7));
        plot(i, DMP.cost_obstacle(end),   sty('m', 'o', [], 'none', 7));
        plot(i, DMP.cost_odometry(end),   sty('g', 'd', [], 'none', 7));
        try
            ylim([0 max(DMP.cost_clean(end-20:end))]); 
        end    
        if i==1
            legend({'total', 'vP', 'startEnd', 'simil', 'obst', 'odom'}, 'Location','northwest'); 
        end

        
        robot.sendTargetCartesianCoordinates(DMP.theta_mean_Frame(10:12)', tr2rpy(DMP.refOrientation(:,:,end)), robot.getHandle('handoverPosition'), 1);
    
        
        figure(h.figcostIK)
        subplot(2,1,1);
        plot(ikerrorTraj);
        subplot(2,1,2);
        plot(i, wIK*robot.costIK_clean(end), styw('k', 'o', [], 'none', 10));
        
        drawnow;    
        i=i+1;            
        % stop criteria
        %errorIK = ikerrorTraj(1) + ikerrorTraj(end);
        %fprintf('\n errorIK (x1000): %g \n', errorIK*1000);
%         if errorIK <= 1e-6 && wIK~=0
%            i = nUpdates+1;  
%         end
%         stopFlag = myKeyPressFcn(hFig);
%         if stopFlag
%            i = nUpdates+1; 
%         end 

%keyboard
    end
       
    
end

function stopFlag = myKeyPressFcn(hFig)

    stopFlag=0;
    try
        figure(hFig);
    catch
        stopFlag=1;        
        fprintf('Stopping loop\n');
        %keyboard
    end
    
    
end

    
    
