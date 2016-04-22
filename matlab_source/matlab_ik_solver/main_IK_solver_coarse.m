

clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);
addpath('./func_robot_vrep');
addpath('./func_robot_vrep/func_vrep_bridge');

%ik = initialize_vrep_darias();
robot = initialize_vrep_baxter('elbow_down');
%robot = initialize_vrep_baxter('elbow_up');


if 0
    if 0 % letter A
        [T, gt] = load_data3D_letterA(nTraj);
    elseif 0
        [T] =  load_data3D_arc(0.25, -0.3, nTraj);
    elseif 1
        [Tc, screwc] =  load_screw_data();    
    end
else
    
end
obst=[];

if 0
    Thandover = ik.readEntityCoordinate('handoverPosition');
    save('testHandover.mat', 'Thandover')
else
    load('testHandover.mat');
end

robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)

    
% create a trajectory from initial pose to final one
% ==================================================
T = robot.goTo(robot.TrestPosture, Thandover, 100);


%% run on each shelf

 k = 1;

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
    if 1% fix trajectory at the start
        DMP{k}.initialGuessStart(vp.init(1:3,4));
        DMP{k}.lockInitialState();
        robot.nTrajConnect = []; % no need to connect trajectories.
    end
    if 1% fix trajectory at the end
        DMP{k}.initialGuessEnd(vp.finl(1:3,4));
        DMP{k}.lockFinalState();    
    end

    DMP{k}.refTraj        = T;
    DMP{k}.refOrientation = robot.goTo(vp.init, vp.finl, numel(T(1,1,:)));

    DMP{k}.automatic_cov_update = 0;

    nUpdates = 5; nRollOut = 25;
    param.fixViaPointPoses   = 1;    % 1: is usually the standard use when the orientations 
                                     % are already given by the task. 0: means
                                     % that the orientation that results from
                                     % the ref. frame search will be used
                                     % instead.
    param.allowDMPShapeChange = 1;  % 1: DMP{k} weights are effective
    param.plotRollOuts=1;
    param.costWeight.similarity  = 1;
    param.costWeight.obstacle    = 0;
    param.costWeight.viaPoint    = 0;
    param.costWeight.viaPointMid = 0;
    param.costWeight.odometry    = 10000000;
    param.costWeight.IK          = 100;
    
    if 0%  k>1 % warm-start of next optimization
        DMP{k-1}.restart;
        DMP{k}.theta_mean_pert_Frame = DMP{k-1}.theta_mean_Frame;
        DMP{k}.theta_mean_pert_dmpx  = DMP{k-1}.theta_mean_dmpx;
        DMP{k}.theta_mean_pert_dmpy  = DMP{k-1}.theta_mean_dmpy;
        DMP{k}.theta_mean_pert_dmpz  = DMP{k-1}.theta_mean_dmpz;  
    end
    
    [DMP{k}] = main_loop(robot, DMP{k},  h, nUpdates, nRollOut, param, 0);


robot.nTraj = 200;
nUpdates = 1; nRollOut = 1;
main_loop(robot, DMP{k},  h, nUpdates, nRollOut, param, 1);



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





