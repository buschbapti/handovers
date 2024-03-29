
clear; clc; close all; dbstop if error;
add_required_paths();
drawnow;

% If this flag is 1 no ROS node is required and poses are generated
% randomly. If the flag is zero, it will wait for a real pose in 
% posesFromROS.txt and for the flag  flagROSfinished.txt
run_without_ROS_trigger = 1;

paramGeneral.initialDT  = 0.5;  % seconds to wait at the first trajectory state, such that Baxter does not jump.
paramGeneral.offsetGripper_humanHand = 0.05 ; % in meters. How close the gripper should get to the hand during the handover.
paramGeneral.tFinal     = [7 13]; % duration of each part of the trajectory
paramGeneral.nTraj      = 150;   % number of steps in each trajectory

% Speed up traj. generation by bypassing vrep.
paramGeneral.speedUpWithoutFKChecking =  1 ;

paramGeneral.debugMode = 0;
paramGeneral.checkFinalSolution = 0; % this will stop the simulation and run 
                                     % the FK on the smoothed final solution
                                     
paramGeneral.dmpExtendTimeFactor = 1.1;    
storePath = '/tmp/matlab_bridge/';
soundPlayer = preloadSound_Baxter;

% create the folder for file exchange
[s,mess,messid] = mkdir(storePath);
% in case the folder already exist clear its content
delete([storePath,'*'])

%% Initialize VREP and lookup table

vrepObj = Vrep( );

% Initialize human demonstrator A (the human that will be improved by REBA)
humanA_R = VrepAgent(vrepObj, 'humanAR');
humanA_R.getDummyHandlesFromVREP({'Dummy_tip_humanA_R'});

humanA_L = VrepAgent(vrepObj, 'humanAL');
humanA_L.getDummyHandlesFromVREP({'Dummy_tip_humanA_L'});

% Initialize Baxter
robot = VrepAgent(vrepObj, 'robot');
robot.simStart;
robot.getDummyHandlesFromVREP({'Dummy_target', 'Dummy_tip', 'Dummy_tipFK'});
                  
                   
robot.opt.floorHeight = -1.220; % meters
robot.opt.tableHeight = -0.475; % meters

robot.requestStreaming; % this is important!

% Update Baxter rest posture
% ============================
load('../../config/baxterRest.mat');
robot.qRestPosture = qRest; % robot.setJointAngles( qRest );
robot.TrestPosture = restT; 
robot.backToRestPosture();

% Setu dummies
% ============================
d_viaPoint = VrepAgent(vrepObj, 'Dummy_viaPoint'); 
d_viaPoint.getDummyHandlesFromVREP({'Dummy_viaPoint_table'});

d_handover = VrepAgent(vrepObj, 'humanFinalHandoverPosition');
d_handover.getDummyHandlesFromVREP({'handoverPosition'});

lookupTraj1 = load_tables('lookupTraj1');

if 0 % new method uses left right trajectories
    lookupTraj2R = load_tables('lookupTraj2'); % old grid with right handovers only
    lookupTraj2.right = lookupTraj2R;
    lookupTraj2L = load_tables_L(); % new grid. Only accounts for traj2 left side
    lookupTraj2.left = lookupTraj2L;
else % previous method, uses only right trajectories
    lookupTraj2 = load_tables('lookupTraj2');
end

default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location

%% Main loop

try % delete the flag just in case
    delete([storePath '/flagROSfinished.txt']);
end


% left 19
mctr = 19; % main loop counter
while 1 % main loop keeps running non-stop    
    tic
    clear posesFromROS; % this is important when doing iterations!!!
    
    fprintf('\n*******************************\n', toc);
    fprintf('Iter %g.\n', mctr);
    fprintf('*********************************\n\n\n', toc);
    
    
    %  Put the scene back to its original configuration
    % ==========================================
    default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location
    load('../../config/baxterRest.mat'); robot.qRestPosture = qRest;  robot.TrestPosture = restT;  robot.backToRestPosture();    
    
    if run_without_ROS_trigger == 1 % skip the ROS part and generate poses randomly
     
        posesFromROS(1,:)   = [0.737216429048	-0.503644892162	-0.418196349287	0.0996327703676	0.994680140491	-0.0242588465096	-0.00981007382333];
        posesFromROS(1,2)   = -0.2+0*posesFromROS(1,2) + 0*0.25*randn;
        posesFromROS        = [posesFromROS; 
                              reba_grid_left_right_20151211(mctr, 'left')];  
                          
        posesFromROS(2,3)   =  posesFromROS(2,3) +0.000;
    else % run for real. This needs ROS node to be run.        
        ctr = 0;
        % check for the presence of the file flag from ROS
        while exist([storePath '/flagROSfinished.txt'], 'file') ==0
            if ~mod(ctr,10000)
                disp('Waiting for ROS to send a pose for handover or via point');
            end
            ctr=ctr+1;
        end
        delete([storePath '/flagROSfinished.txt']);
        fprintf('\n\nROS poses acquired.\n');
        pause(0.1);
        load([storePath '/posesFromROS.txt']);
        
        posesFromROS(2,3)   =  posesFromROS(2,3) + 0.02;
    end
        
    fprintf('Poses from ROS received\n');

    % fix quaternion representation from Matlab to ROS
    posesMatlabFormat = changeQuaternionOrder(posesFromROS);
        
    if posesFromROS(3,1) == 1 % right handed
        humanHand = humanA_R;
        humanA_L.sendTargetCartesianCoordinates([1 -1 -1.2], [0 0 0], humanA_L.getHandle('Dummy_tip_humanA_L'), 1);
        if isfield(lookupTraj2, 'right')
            lookupTraj2Final = lookupTraj2.right;
        else % if there is no library for left handover, use the right one
            lookupTraj2Final = lookupTraj2;
        end        
    else % left hand
        humanHand = humanA_L;
        humanA_R.sendTargetCartesianCoordinates([1 -0.8 -1.2], [0 0 0], humanA_R.getHandle('Dummy_tip_humanA_R'), 1);
        if isfield(lookupTraj2, 'left')
            lookupTraj2Final = lookupTraj2.left;
        else % if there is no library for left handover, use the right one
            lookupTraj2Final = lookupTraj2;
        end
    end

    % main optimization code runs here
    fprintf('Wait while trajectory is being generated...\n');
    [traj1, traj2] = get_robot_trajectory(posesMatlabFormat, robot, d_viaPoint, ...
                                          d_handover, humanHand, paramGeneral,...
                                          lookupTraj1, lookupTraj2Final, soundPlayer);
    
    % skip if solution does not right in vrep
    if 1 % ~isempty(traj1)
        write_trajectory_file(storePath, traj1, traj2, paramGeneral.nTraj, paramGeneral.initialDT);
        fprintf('\n*******************************\n', toc);
        fprintf('Trajectory generated %g sec.\n', toc);
        fprintf('*********************************\n\n\n', toc);
    end
    mctr = mctr+3;        
    close all; 
    
    
end







