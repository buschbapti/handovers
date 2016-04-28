

clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);
addpath('./func_robot_vrep');
addpath('./func_robot_vrep/func_vrep_bridge');

%ik = initialize_vrep_darias();
robot = initialize_vrep_baxter('elbow_down');
%robot = initialize_vrep_baxter('elbow_up');
%system('~/projects/vrep/V-REP_PRO_EDU_V3_2_2_64_Linux/./vrep.sh   ../matlab_main/myScene.ttt &');





%TendEff = create_endeffector_positions_insertion_toy(robot);

obst=[];
if 0
    Thandover = robot.readEntityCoordinate('handoverPosition');
    save('testHandover.mat', 'Thandover')
else
    load('testHandover.mat');
end

Thandover(1,4) = Thandover(1,4)+0.5;
Thandover(2,4) = Thandover(2,4)+0.55;
% Thandover(3,4) = Thandover(3,4)-0.0;
%Thandover= Thandover*se3(0, d2r(-90), 0, 0, 0, 0);

robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)


if Thandover(1,4) <= 0.75
    Thandover(1,4) = 0.75;
end

% create a trajectory from initial pose to final one
% ==================================================
T = robot.goTo(robot.TrestPosture, Thandover, 100);


%% run on each shelf

    k = 1;
    robot.nTraj = 10;
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
    DMP{k} = Isoemp_dmp_task(vp, obst, nTraj);

    % keep trajectory floating at intial and final states
    DMP{k}.initialGuessStart(vp.init(1:3,4) );
    DMP{k}.initialGuessEnd(vp.finl(1:3,4) )  ;
    if 1  % fix trajectory at the start
        DMP{k}.initialGuessStart(vp.init(1:3,4));
        DMP{k}.lockInitialState();
        robot.nTrajConnect = []; % no need to connect trajectories.
    end
    if 0  % fix trajectory at the end
        DMP{k}.initialGuessEnd(vp.finl(1:3,4));
        DMP{k}.lockFinalState();    
    end

    DMP{k}.refTraj        = T;
    DMP{k}.refOrientation = robot.goTo(vp.init, vp.finl, numel(T(1,1,:)));

    DMP{k}.automatic_cov_update = 0;

    nUpdates = 20 ;
    nRollOut = 10;
    param.fixViaPointPoses   = 1;    % 1: is usually the standard use when the orientations 
                                     % are already given by the task. 0: means
                                     % that the orientation that results from
                                     % the ref. frame search will be used
                                     % instead.
    param.allowDMPShapeChange = 1;   % 1: DMP{k} weights are effective
    param.plotRollOuts=1;
    param.costWeight.similarity  = 0;
    param.costWeight.obstacle    = 0;
    param.costWeight.startGoal   = 0;
    param.costWeight.viaPointMid = 0;
    param.costWeight.odometry    = 1e3;
    param.costWeight.objectXdist = 0; % penalize object for being close to the robot
    param.costWeight.IK          = 1e9;
    
    if 0%  k>1 % warm-start of next optimization
        DMP{k-1}.restart;
        DMP{k}.theta_mean_pert_Frame = DMP{k-1}.theta_mean_Frame;
        DMP{k}.theta_mean_pert_dmpx  = DMP{k-1}.theta_mean_dmpx;
        DMP{k}.theta_mean_pert_dmpy  = DMP{k-1}.theta_mean_dmpy;
        DMP{k}.theta_mean_pert_dmpz  = DMP{k-1}.theta_mean_dmpz;  
    end
    
    [DMP{k}] = main_loop(robot, DMP{k},  h, nUpdates, nRollOut, param, 0);

    
% h = restartFigure( [], T, obst, vp, robot);
% robot.nTraj = 10;
% nUpdates = 5; nRollOut = 10;
% main_loop(robot, DMP{k},  h, nUpdates, nRollOut, param,0);

DMP{k}.restart;
h = restartFigure( [], T, obst, vp, robot);
robot.nTraj = 200;
nUpdates = 1; nRollOut = 1;
main_loop(robot, DMP{k},  h, nUpdates, nRollOut, param, 1);




break

% get current position and use as rest posture
param.hfig = gcf;
homogTransfPlot( DMP{k}.TrollOut, param)


Thandover =  DMP{k}.TrollOut(:,:,end)





break


DMP{k}.restart;
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
main_loop(ik, DMP{k},  h, nUpdates, nRollOut, param, 1);


% 





