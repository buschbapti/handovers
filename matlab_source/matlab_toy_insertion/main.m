if 1
    %initialize_dropbox_path(1,0,1);
    addpath('../func_misc_self_contained');
    addpath('../func_kinematics');
    addpath('./func_robot_vrep');
    addpath('./func_robot_vrep/func_vrep_bridge');
    addpath('../matlab_main/func_generalize_traj');
    addpath('../matlab_main/audio');
end

clear; clc; close all; dbstop if error;

% If this flag is 1 no ROS node is required and poses are generated
% randomly. If the flag is zero, it will wait for a real pose in 
% posesFromROS.txt and for the flag  flagROSfinished.txt
paramGeneral.run_without_ROS_trigger = 1;

% Which type of grid you want to load
% 'reba' or 'relative'
paramGeneral.grid_type = 'reba'; 


% How to generalize
% 1: Generalize DMP, but it can be risky
% 2: Nearest neighbour does not generalize but use the closes computed solution
paramGeneral.generalizeMethod = 1;

paramGeneral.initialDT  = 0.5;  % seconds to wait at the first trajectory state, such that Baxter does not jump.
paramGeneral.offsetGripper_humanHand = 'not in use'; % in meters. How close the gripper should get to the hand during the handover.
paramGeneral.tFinal     = [7]; % duration of each part of the trajectory
paramGeneral.nTraj      = 200;    % number of steps in each trajectory

% Speed up traj. generation by bypassing vrep.
paramGeneral.speedUpWithoutFKanimation =  1 ;

paramGeneral.debugMode = 0;
paramGeneral.checkFinalSolution = 0; % this will stop the simulation and run 
                                     % the FK on the smoothed final solution

% Forces the position of the end-effector to be at least "minDistFromRobot"
% away from the origin. Empirically I think anything under 0.5 will have a
% high probability of collision or unnatural IK solutions
paramGeneral.minMaxDistFromRobot = [0.7  1.0];

paramGeneral.dmpExtendTimeFactor = 'not in use'; 

paramGeneral.plotGrid = 0;

%%
storePath = '/tmp/matlab_bridge/';


% create the folder for file exchange
[s,mess,messid] = mkdir(storePath);
% in case the folder already exist clear its content
delete([storePath,'*'])

% Initialize VREP and lookup table
% =================================================
robot = initialize_vrep_baxter('elbow_down');
tr = GeneralizeTrajectory(paramGeneral);
clear paramGeneral;
tr.sceneBackToOriginalPosition(robot);

%% Main loop

try % delete the flag just in case
    delete([storePath '/flagROSfinished.txt']);
end

mctr=1;

while mctr <  20  % main loop keeps running non-stop
    
    tic
    clear posesFromROS; % this is important when doing iterations!!!
    
    fprintf('\n*******************************\n', toc);
    fprintf('Iter %g.\n', mctr);
    fprintf('*********************************\n\n\n', toc);
    
    tr.sceneBackToOriginalPosition(robot);
    
    if tr.param.run_without_ROS_trigger == 1 % skip the ROS part and generate poses randomly       
        posesFromROS = tr.createFakePose(15, 0.1, randi([1 5]), []);
        %posesFromROS = tr.createFakePose(5, 0.1, 5, [1  0  0.27]);
                                       %(noiseDeg, noiseCart, rotIndex, xyz)        
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
        fprintf('\n\nROS poses acquuired.\n');
        pause(0.1);
        load([storePath '/posesFromROS.txt']);
    end
    posesFromROS(2,1) = tr.forceHumanAwayFromRobot(posesFromROS(2,1));
        
    if 1
        save('tempPose.mat', 'posesFromROS');
    else
        load('tempPose.mat');
    end
    
    if 0
        save('tempPosePat.mat', 'posesFromROS');
    elseif 0
        load('tempPosePat.mat');
    end    
    
    fprintf('Poses from ROS received\n');
    posesMatlabFormat = tr.changeQuaternionOrder(posesFromROS);

    tr.makeRobotTrajectory(posesMatlabFormat, robot);   
    
    % skip if solution does not right in vrep
    tr.write_trajectory_file(storePath);
    fprintf('\n*******************************\n', toc);
    fprintf('Trajectory generated %g sec.\n', toc);
    fprintf('*********************************\n\n\n', toc);

    mctr = mctr+1;
    
    toc
    %pause
    tr.resetForNexIteration();      
    
    close all
end

break;

for t =1:numel(tr.qTraj1(:,1))
    robot.setJointAngles(tr.qTraj1(t,:),1);
end




