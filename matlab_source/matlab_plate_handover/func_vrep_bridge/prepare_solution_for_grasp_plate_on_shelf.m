function [traj1, graspGoto, traj2] = prepare_solution_for_grasp_plate_on_shelf(robot, demo, sol1, sol2)
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

if 0% ~strcmp(demo.typeOfCollaboration, 'grasp plate on shelf')
    error('You are processing the wrong type of task here.');
end

% Plot again here
param.hfig = plot_workspace();
param.axis_length = 0.145;
param.nMaxPlots = 50;
param.shadowHeight     = -1.22;
param.axesPlotStyle{1} = struct('LineWidth', 5.1, 'Color', [1 .0075 .0075]);
param.axesPlotStyle{2} = struct('LineWidth', 5.1, 'Color', [.0075 1 .0075]);
param.axesPlotStyle{3} = struct('LineWidth', 5.1, 'Color', [.0075 .0075 1]);
param.pathPlotStyle    = struct('LineWidth', 5, 'Color', 'k', 'Marker', 'o');
homogTransfPlot(demo.part{1}.newVP.T, param);
homogTransfPlot(sol2.T(:,:,end), param);

param.axis_length = 0.045;
param.nMaxPlots = 10;
param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .0075 .0075]);
param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.0075 1 .0075]);
param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.0075 .0075 1]);

param.pathPlotStyle    = struct('LineWidth', 2,   'Color', 'b', 'Marker', 'none');
homogTransfPlot(sol1.T, param);
param.pathPlotStyle    = struct('LineWidth', 2,   'Color', 'r', 'Marker', 'none');
homogTransfPlot(sol2.T, param);

% approaching
add_joint_trajectory(sol1.T(:,:,1), robot); % this is to set the initial position before executing the trajectory
traj1 = add_joint_trajectory(sol1.T, robot);
plot3( traj1.p(1,:), traj1.p(2,:), traj1.p(3,:), style('b', 'o', 1, '-', 7) );


% grasping goto
h_ =  figurewe('graspingGoTo');
xlabel 'x'; ylabel 'y'; zlabel 'z';
angles = homogTransfMatrixProjectedAngles(traj1.T(:,:,end), h_ );   
[~,~,~,T_] =  moveStateOnCartesianPlane(traj1.T(:,:,end), 'xy', d2r(-90), linspace(0.01,0.10,10) );
homogTransfPlot(T_, param);
graspGoto = add_joint_trajectory(T_, robot);

% handover
figure(param.hfig);
add_joint_trajectory(sol2.T(:,:,1), robot); % this is to set the initial position before executing the trajectory
traj2 = add_joint_trajectory(sol2.T, robot);
plot3( traj2.p(1,:), traj2.p(2,:), traj2.p(3,:), style('r', 'o', 1, '-', 7) );




end

function [traj] = add_joint_trajectory(T, robot)

    % get the joint trajectories by calling vrep and running the simulation
    [Trobot, qrobot] = replay_solution_until_damping_disappears(T, robot);

    traj.p = squeeze(Trobot(1:3,4,:));
    traj.T = Trobot(:, :, :);
    traj.q = qrobot(: , :);



end















