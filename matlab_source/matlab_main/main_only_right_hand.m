% Important transformation!!
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


% Speed up traj. generation by bypassing vrep.
paramGeneral.speedUpWithoutFKChecking =  1 ;

paramGeneral.debugMode = 0;
paramGeneral.checkFinalSolution = 01; % this will stop the simulation and run 
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


lookupTraj1 = load_tables('lookupTraj1');
lookupTraj2 = load_tables('lookupTraj2');


default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location

%% Main loop

try % delete the flag just in case
    delete([storePath '/flagROSfinished.txt']);
end

mctr = 1; % main loop counter


while 1 % main loop keeps running non-stop
    
    tic

    %  Put the scene back to its original configuration
    % ==========================================
    default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location
    load('../../config/baxterRest.mat'); robot.qRestPosture = qRest;  robot.TrestPosture = restT;  robot.backToRestPosture();    
    
    if run_without_ROS_trigger == 1 % skip the ROS part and generate poses randomly
     
        if 1 % random generator
            scale = 1;
            placeHolderParam.viaPoint.stdPos = scale*[0.5 0.5  0.5];    % meters
            placeHolderParam.viaPoint.stdRot = scale*d2r([45  45  45]); % radians world coordinates
            placeHolderParam.handOver.stdPos = scale*[0.995 0.8 0.85];
            placeHolderParam.handOver.stdRot = scale*d2r([ 0 0 0]);
            placeHolderParam.deterministic   = 0; % make sure the shift is exact. Otherwise use it as std noise.
        end
        if 0 % deterministic placement
            scale = 1;
            placeHolderParam.viaPoint.stdPos = scale*[0.0   -0.2   0];    % meters
            placeHolderParam.viaPoint.stdRot = scale*d2r([0  0    0]); % radians world coordinates
            placeHolderParam.handOver.stdPos = scale*[0.5  0  0];
            placeHolderParam.handOver.stdRot = scale*d2r([ 0  0  +0]);
            placeHolderParam.deterministic   = 1; % make sure the shift is exact. Otherwise use it as std noise.
        end       
        
        [posesFromROS, tmpvp, tmpreba] = placeholder_get_positions(d_viaPoint, d_handover, placeHolderParam);
        
        fprintf('Current REBA poses\n');
        fprintf('L1  %g %g %g %g %g %g %g\n', posesFromROS(1,:));
        fprintf('L2  %g %g %g %g %g %g %g\n', posesFromROS(2,:));
        
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
    end
        
    fprintf('Poses from ROS received\n');

    % fix quaternion representation from Matlab to ROS
    posesMatlabFormat = changeQuaternionOrder(posesFromROS);
        
    % main optimization code runs here
    fprintf('Wait while trajectory is being generated...\n');
    [traj1, traj2] = get_robot_trajectory(posesMatlabFormat, robot, d_viaPoint, ...
                                          d_handover, humanA, paramGeneral,...
                                          lookupTraj1, lookupTraj2, soundPlayer);
    
    write_trajectory_file(storePath, traj1, traj2, paramGeneral.nTraj, paramGeneral.initialDT);
    fprintf('\n****************************\n', toc);
    fprintf('Trajectory generated %g sec.\n', toc);
    fprintf('\n****************************\n\n\n', toc);
    mctr=mctr+1;
    close all;   
    
end


%robot.setJointAngles(d2r([ 95 -60 0 0 0 0 0]),1);














