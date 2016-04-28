

clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);
addpath('./func_robot_vrep');
addpath('./func_robot_vrep/func_vrep_bridge');

%ik = initialize_vrep_darias();
robot = initialize_vrep_baxter('elbow_down');
%robot = initialize_vrep_baxter('elbow_up');
%system('~/projects/vrep/V-REP_PRO_EDU_V3_2_2_64_Linux/./vrep.sh   ../matlab_main/myScene.ttt &');


TendEff = create_endeffector_positions_insertion_toy();

if 0
    robot.sendTargetCartesianCoordinates(TendEff{1}(1:3,4), tr2rpy(TendEff{1}), robot.getHandle('Dummy_target'), 1)
    robot.sendTargetCartesianCoordinates(TendEff{2}(1:3,4), tr2rpy(TendEff{2}), robot.getHandle('Dummy_target'), 1)
    robot.sendTargetCartesianCoordinates(TendEff{3}(1:3,4), tr2rpy(TendEff{3}), robot.getHandle('Dummy_target'), 1)
    robot.sendTargetCartesianCoordinates(TendEff{4}(1:3,4), tr2rpy(TendEff{4}), robot.getHandle('Dummy_target'), 1)
    robot.sendTargetCartesianCoordinates(TendEff{5}(1:3,4), tr2rpy(TendEff{5}), robot.getHandle('Dummy_target'), 1)
end


obst=[];

% run on each position
% =======================================
load('sol.mat');
for k =  4%numel(sol)+1: numel(TendEff)
    
    
    Thandover = TendEff{k};
    
    robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)
    if Thandover(1,4) <= 0.75
        Thandover(1,4) = 0.75;
    end

    % create a trajectory from initial pose to final one
    % ==================================================
    T = robot.goTo(robot.TrestPosture, Thandover, 100);

    
    robot.nTraj = 5;
    robot.nTrajConnect = 1;

    nTraj = size(T,3);

    % get good initial guess
    % T = improve_initial_guess(T, screwc{k}.endEffPos, robot);

    % defining via points
    vp.init = robot.TrestPosture;
    vp.finl = T(:,:,end);

    robot.sendTargetCartesianCoordinates(vp.finl(1:3,4), tr2rpy(vp.finl(:,:)), robot.getHandle('handoverPosition'), 1);

    h = createFig( [], squeeze(T(1:3,4,:)), [], vp);


    %% creating dmp ikrobot
    DMP = Isoemp_dmp_task(vp, obst, nTraj);

    % keep trajectory floating at intial and final states
    DMP.initialGuessStart(vp.init(1:3,4) );
    DMP.initialGuessEnd(vp.finl(1:3,4) )  ;
    if 1  % fix trajectory at the start
        DMP.initialGuessStart(vp.init(1:3,4));
        DMP.lockInitialState();
        robot.nTrajConnect = []; % no need to connect trajectories.
    end
    if 0  % fix trajectory at the end
        DMP.initialGuessEnd(vp.finl(1:3,4));
        DMP.lockFinalState();    
    end

    DMP.refTraj        = T;
    DMP.refOrientation = robot.goTo(vp.init, vp.finl, numel(T(1,1,:)));

    DMP.automatic_cov_update = 0;

    nUpdates = 20 ;
    nRollOut = 10;
    param.fixViaPointPoses   = 1;    % 1: is usually the standard use when the orientations 
                                     % are already given by the task. 0: means
                                     % that the orientation that results from
                                     % the ref. frame search will be used
                                     % instead.
    param.allowDMPShapeChange = 1;   % 1: DMP weights are effective
    param.plotRollOuts=1;
    param.costWeight.similarity  = 0;
    param.costWeight.obstacle    = 0;
    param.costWeight.startGoal   = 1e3;
    param.costWeight.viaPointMid = 0;
    param.costWeight.odometry    = 0;
    param.costWeight.objectXdist = 0; % penalize object for being close to the robot
    param.costWeight.IK          = 1e9;
    
    if k > 1 % warm-start of next optimization
        DMP.theta_mean_pert_Frame = sol{k-1}.DMP.theta_mean_Frame;
        DMP.theta_mean_pert_dmpx  = sol{k-1}.DMP.theta_mean_dmpx;
        DMP.theta_mean_pert_dmpy  = sol{k-1}.DMP.theta_mean_dmpy;
        DMP.theta_mean_pert_dmpz  = sol{k-1}.DMP.theta_mean_dmpz;
        DMP.theta_Cov_Frame       = DMP.theta_Cov_Frame*0.05;
    end
    
    [DMP] = main_loop(robot, DMP,  h, nUpdates, nRollOut, param, 0);

    DMP.restart;
    h = restartFigure( [], T, obst, vp, robot);
    robot.nTraj = 200;
    [~, T, q] = main_loop(robot, DMP,  h, nUpdates, nRollOut, param, 1);
    
keyboard    
    sol{k}.TendEff = TendEff{k};
    sol{k}.DMP = DMP;
    sol{k}.T = T;
    sol{k}.q = q;
    sol{k}.param = param;
    
    save('sol.mat', 'sol');
end

k=4
paramFilter.filterOrder = 4; 
paramFilter.filterFreq = 0.05;
for j=1:7
    q(:,j) = smoothf( sol{k}.q(:,j), paramFilter );
end
figurew
plot(sol{k}.q, 'Color', [0.8 0.8 0.8], 'LineWidth', 4)
plot(q)

for t = 1:numel(q(:,1))
    robot.setJointAngles(q(t,:), 1)
    pause(0.01)
end


break

% get current position and use as rest posture
param.hfig = gcf;
homogTransfPlot( DMP.TrollOut, param)


Thandover =  DMP.TrollOut(:,:,end)





break


DMP.restart;
h = restartFigure( [], T, obst, vp, robot);


% 
% for k=1:10
%     T = improve_initial_guess(T, screwc{k}.endEffPos, ik);
% end
% 


k = 1;    
T     = Tc{k};
nTraj = size(T,3);
h = restartFigure( [], T, obst, vp, ik);
load(['DMP_' num2str(k) 'to' num2str(k) '_coarse.mat']);

    % defining via points
    vp.init = ik.TrestPosture;
    vp.finl = screwc{k}.endEffPos;

ik.nTraj = nTraj;
nUpdates = 1; nRollOut = 1;
main_loop(ik, DMP,  h, nUpdates, nRollOut, param, 1);


% 





