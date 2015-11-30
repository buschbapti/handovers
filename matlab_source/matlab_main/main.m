% Two important transformations!!
%
% => calibrate_ref_TUDa_Inria_ref_frame.m
%     This is a calibration function to account for the differences between the locations
%     of the world frames of Darias and Baxter. This is because the scene in VREP assumes
%     Baxter is located in the world of Darias.
%     From our previous talks, I assume that the XY location and the orientation 
%     of the reference frames are the same. But the Z height is definitely different. 
%     As an initial guess I think the reference of Darias is 0.3543 meters above the 
%     reference frame of Baxter
%
%
% =>  changeQuaternionOrder.m
%     The code assumes the poses from ROS are given in the roder
%     [x y z, quatx, quaty, quatz, quatScale]
%     The function changeQuaternionOrder will change the order to fit the
%     matlab code, which is
%     [x y z, quatScale, quatx, quaty, quatz]
%
%
% To run the code without ROS you can set the flag
%  run_without_ROS_trigger = 1
% 
% Then a fake provider of ROS poses will generate random positions and
% orientations for both via point and handover locations.
%
clear; clc; close all; dbstop if error;
add_required_paths();
drawnow;

% If this flag is 1 no ROS node is required and poses are generated
% randomly. If the flag is zero, it will wait for a real pose in 
% posesFromROS.txt and for the flag  flagROSfinished.txt
run_without_ROS_trigger = 1;

paramGeneral.initialDT = 0.5;  % seconds to wait at the first trajectory state, such that Baxter does not jump.
paramGeneral.offsetGripper_humanHand = 0.05 ; % in meters. How close the gripper should get to the hand during the handover.
paramGeneral.tFinal    = [10 10]; % duration of each part of the trajectory
paramGeneral.nTraj     = 150;   % number of steps in each trajectory



paramGeneral.debugMode = 0;
paramGeneral.checkFinalSolution = 1; % this will stop the simulation and run 
                                     % the FK on the smoothed final solution
                                     
paramGeneral.dmpExtendTimeFactor = 1.1;    
storePath = '/tmp/matlab_bridge/';
soundPlayer = preloadSound_Baxter;

% create the folder for file exchange
[s,mess,messid] = mkdir(storePath);
% if the folder already exist clear its content
if messid == 'MATLAB:MKDIR:DirectoryExists'
    delete([storePath,'*'])
end

%% Initialize VREP and lookup table

vrepObj = Vrep( );

% Initialize human demonstrator A (the human that will be improved by REBA)
humanA = VrepAgent(vrepObj, 'human');
humanA.getDummyHandlesFromVREP({'Dummy_tip_humanA_R'});

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


if 1% load lookup table
    lookupTraj1 = load_tables('lookupTraj1');
    lookupTraj2 = load_tables('lookupTraj2');
else
    lookupTraj1 = load_tables_debug('lookupTraj1');
    lookupTraj2 = load_tables_debug('lookupTraj2');    
end

default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location
    

%% Main loop

try % delete the flag just in case
    delete([storePath '/flagROSfinished.txt']);
end

mctr = 1; % main loop counter


while 1 % main loop keeps running non-stop

    %  Put the scene back to its original configuration
    % ==========================================
    default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location
    load('../../config/baxterRest.mat'); robot.qRestPosture = qRest;  robot.TrestPosture = restT;  robot.backToRestPosture();    
    
    if run_without_ROS_trigger == 1 % skip the ROS part and generate poses randomly
     
        if 0 % random generator
            scale = 0.85;
            placeHolderParam.viaPoint.stdPos = scale*[0.6 0.6  1];    % meters
            placeHolderParam.viaPoint.stdRot = scale*d2r([45  45  180]); % radians world coordinates
            placeHolderParam.handOver.stdPos = scale*[0.6  1  1];
            placeHolderParam.handOver.stdRot = scale*d2r([ 45  45  45]);
            placeHolderParam.deterministic   = 0; % make sure the shift is exact. Otherwise use it as std noise.
        end
        if 1 % deterministic placement
            scale = 1;
            placeHolderParam.viaPoint.stdPos = scale*[0.1  0.05   -0.1];    % meters
            placeHolderParam.viaPoint.stdRot = scale*d2r([0  0 -45]); % radians world coordinates
            placeHolderParam.handOver.stdPos = scale*[0.1  0.0  0];
            placeHolderParam.handOver.stdRot = scale*d2r([ 0  0  +0]);
            placeHolderParam.deterministic   = 1; % make sure the shift is exact. Otherwise use it as std noise.
        end
        
        [posesFromROS, tmpvp, tmpreba] = placeholder_get_positions(d_viaPoint, d_handover, placeHolderParam);

        if 0%~mod(mctr,3)
             posesFromROS(1,:) = -999*ones(1,7);
        end

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
        fprintf('\n\nROS poses acquired.\n')
        pause(1);
        
        load([storePath '/posesFromROS.txt']);
        %posesFromROS = calibrate_ref_TUDa_Inria_ref_frame(posesFromROS);
    end

        
    fprintf('Poses from ROS received\n');
    %play_sound(soundPlayer, 'poses_acquired'); pause(2);
    
    %play_sound(soundPlayer, 'generating_trajectory'); pause(1);
    % fix quaternion representation from Matlab to ROS
    posesMatlabFormat = changeQuaternionOrder(posesFromROS);
        
    % main optimization code runs here
    fprintf('Wait while trajectory is being generated...\n');
    [traj1, traj2] = get_robot_trajectory(posesMatlabFormat, robot, d_viaPoint, ...
                                          d_handover, humanA, paramGeneral,...
                                          lookupTraj1, lookupTraj2, soundPlayer);
    
    write_trajectory_file(storePath, traj1, traj2, paramGeneral.nTraj, paramGeneral.initialDT);
    fprintf('Trajectory generated and sent to ROS!!\n\n\n');

    
    pauseTime = 0.0001;
    fprintf('*** Repeating a new cycle in %g seconds ***\n\n\n', pauseTime);    
    
    pause(pauseTime);

    mctr=mctr+1;
    close all;   
 
    
end
















