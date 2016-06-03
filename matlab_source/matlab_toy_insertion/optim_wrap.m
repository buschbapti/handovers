function sol = optim_wrap(robot, Thandover, sol)

    obst=[];

    % create a trajectory from initial pose to final one
    % ==================================================
    T = robot.goTo(robot.TrestPosture, Thandover, 200);

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

    nUpdates = 20;  nRollOut = 7; 
    
    %nUpdates = 2;  nRollOut = 2; 
    
    
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

    if ~isempty(sol)  % warm-start of next optimization
        DMP.theta_mean_pert_Frame = sol.DMP.theta_mean_Frame;
        DMP.theta_mean_pert_dmpx  = sol.DMP.theta_mean_dmpx;
        DMP.theta_mean_pert_dmpy  = sol.DMP.theta_mean_dmpy;
        DMP.theta_mean_pert_dmpz  = sol.DMP.theta_mean_dmpz;
        DMP.theta_Cov_Frame       = DMP.theta_Cov_Frame*0.5;
    end

    [DMP] = main_loop(robot, DMP,  h, nUpdates, nRollOut, param, 0);

    DMP.restart;
    h = restartFigure( [], T, obst, vp, robot);
    robot.nTraj = 200;
    [~, T, q] = main_loop(robot, DMP,  h, nUpdates, nRollOut, param, 1);
    
    sol.TendEffOriginal = Thandover;
    sol.DMP = DMP;
    sol.T = T;
    sol.q = q;
    sol.param = param;
    
    
end





