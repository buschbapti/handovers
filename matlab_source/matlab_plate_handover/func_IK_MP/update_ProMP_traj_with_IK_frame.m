function [newT, runMP, param] = update_ProMP_traj_with_IK_frame(runMP, sysIK, param, plotShadowzheight, noiseScale )
% function [runMP, param, hd] = update_ProMP_traj_with_IK_frame(runMP, sysIK, param, robot.vrepm.opt. )
%
% This function is *specific for each task*. It will move the perturbed ProMP trajectory 
% to a reference frame location that is to be perturbed.
%
% INPUT
%   runMP: the perturbed MP solution that is still based on the original
%          location of the reference frame
%
%   sysIK: control parameters of the IK.
    

    % Move the MP trajectory to the current IK solution frame
    % ===================================================
    nTraj = numel(runMP.dof(1).q);
    
    % 1. Get MP solution on initial reference frame
    for t =1:nTraj
        oldMP.T(:,:,t) = [eye(3,3)  [runMP.dof(1).q(t); runMP.dof(2).q(t);  runMP.dof(3).q(t)] ;[0 0 0 1]] ;
        oldMP.p(:,t)   = [runMP.dof(1).q(t); runMP.dof(2).q(t);  runMP.dof(3).q(t) ] ;     
    end
    
    % 2. Update the reference frame according to the value in sysIK.dof.wMean
    runIK = roll_out_refFrame([], sysIK, oldMP, [], param.frameRotateCenter, noiseScale);
    
    
    dbg = 0;
    if dbg        
        hw = figurewe;
        quickPlotHomogTransTrajectories(hw, oldMP.T, runIK{1}{1}.T);        
        aT = runIK{1}{1}.T;
    end
    
    % 3. This imposes the orientation of the desired solution.
    %    This is a trick that force the solution to have the desire
    %    orientation. It needs the orientation of the first and last frame
    %    at least, such that orientations will be interpolated. You can
    %    also specify via-points, where each will have its own orientation.
    runIK{1}{1} = interpolate_poses(runIK{1}{1}, param.mp.viaPoint.indexes, param.mp.viaPoint.T);

    % the MP solution on the new IK frame
    newT = runIK{1}{1};
    
    if dbg
        hw=figurewe;
        quickPlotHomogTransTrajectories(hw, aT, runIK{1}{1}.T);
        keyboard
    end   
    
    % Update the xyz trajectory
    % There was a bug here 
    %     runMP.dof(2).q = newT.p(1,:)';  <=== BUG in previous versions!!!
    runMP.dof(1).q = newT.p(1,:)';
    runMP.dof(2).q = newT.p(2,:)';
    runMP.dof(3).q = newT.p(3,:)';
    
    % Plot the trajectory
    if ~isempty(param.hfig.single_IK_iteration)
        figure(param.hfig.single_IK_iteration);
        try delete(param.hfig.single_IK_iterationTraj); end
        try delete(param.hfig.single_IK_iterationShadow); end

        [htraj2, hshadow2] = plot_traj_with_shadow( oldMP.p, plotShadowzheight, sty('b',  [], 2, '--'), [] ) ;
        [htraj1, hshadow1] = plot_traj_with_shadow( newT.p,  plotShadowzheight, styw('m', [], 2, '-', 5), [] ) ;    
        legend({'initial', 'MP', 'MPIK'});
        drawnow
   
        param.hfig.single_IK_iterationTraj   = [htraj1  htraj2];
        param.hfig.single_IK_iterationShadow = [hshadow1  hshadow2];
    end
     
end


















