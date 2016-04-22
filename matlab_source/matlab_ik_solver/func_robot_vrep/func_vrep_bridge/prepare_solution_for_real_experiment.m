function [traj1, traj2] = prepare_solution_for_real_experiment(robot, sol1, sol2, plotFlag)
% This function must only be called to process solutions of the type
% of grasping the plate on the shelf. 
% It uses a heuristic that assumes that the new shelf position will be 
% approximately parallel to the wall at the right side of Darias
%
%
% OUTPUT
% vrepTraj = 
% 
%      T: [4x4x100 double]
%      p: [3x100 double]
%      q: [100x7 double]
%      dt: 0.0232

    if plotFlag
        
        % Plot again here
        param.hfig = plot_workspace();
        param.axis_length = 0.075;
        param.nMaxPlots = 25;
        param.shadowHeight     = -1.22;
        param.axesPlotStyle{1} = struct('LineWidth', 3, 'Color', lightRGB(1));
        param.axesPlotStyle{2} = struct('LineWidth', 3, 'Color', lightRGB(2));
        param.axesPlotStyle{3} = struct('LineWidth', 3, 'Color', lightRGB(3));
        param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'k', 'Marker', 'o');
        homogTransfPlot(sol1.T, param);


        param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'r', 'Marker', 'o');
        homogTransfPlot(sol2.T, param);
    end

    traj1 = add_joint_trajectory_withoutIK(sol1, robot);
    traj2 = add_joint_trajectory_withoutIK(sol2, robot);
        
    if 0 % old method relies on IK
        traj1 = add_joint_trajectory(sol1.T, robot);
        traj2 = add_joint_trajectory(sol2.T, robot);
    end

end

function [sol] = add_joint_trajectory_withoutIK(sol, robot)

    % get the joint trajectories by calling vrep and running the simulation
    % [Trobot, qrobot] = replay_solution_until_damping_disappears(T, robot);

    nExtrasStart = 15;
    nExtrasEnd   = 15;    
    
    solOr = sol;
    
    % add some extra frames at the beginning
    for j = 1:nExtrasStart
        sol.T = cat(3, sol.T(:,:,1), sol.T);
        sol.q = [sol.q(1,:); sol.q];
    end
        
    % add some extra frames at the end
    for j = 1:nExtrasEnd
        sol.T = cat(3, sol.T, sol.T(:,:,end));
        sol.q = [sol.q; sol.q(end,:)];
    end    
    
    
    sol.p = squeeze(sol.T(1:3,4,:));
    
end


function [traj] = add_joint_trajectory(T, robot)

    % get the joint trajectories by calling vrep and running the simulation
    [Trobot, qrobot] = replay_solution_until_damping_disappears(T, robot);

    traj.p = squeeze(Trobot(1:3,4,:));
    traj.T = Trobot(:, :, :);
    traj.q = qrobot(: , :);

end















