

clear; clc; close all; dbstop if error;
initialize_dropbox_path(1, 1);
drawnow;

paramGeneral.debugMode = 0;

%% Initialize vrep object.
vrepObj = Vrep( );

% Initialize human demonstrator A (the human that will be improved by REBA)
humanA = VrepAgent(vrepObj, 'human');
humanA.getDummyHandlesFromVREP({'Dummy_tip_humanA_R'});

% Initialize Baxter
robot = VrepAgent(vrepObj, 'robot');
robot.simStart;
robot.getDummyHandlesFromVREP({'Dummy_target', 'Dummy_tip'});
robot.getJointHandles({'Baxter_rightArm_joint1', 'Baxter_rightArm_joint2', 'Baxter_rightArm_joint3', 'Baxter_rightArm_joint4', ...
                        'Baxter_rightArm_joint5', 'Baxter_rightArm_joint6', 'Baxter_rightArm_joint7'});

robot.opt.floorHeight = -1.220; % meters
robot.opt.tableHeight = -0.475; % meters

robot.requestStreaming; % this is important!

% Update Baxter rest posture
% ============================
load('baxterRest.mat');
robot.qRestPosture = qRest; % robot.setJointAngles( qRest );
robot.TrestPosture = restT; 
robot.backToRestPosture();

% Setu dummies
% ============================
d_viaPoint = VrepAgent(vrepObj, 'Dummy_viaPoint'); 
d_viaPoint.getDummyHandlesFromVREP({'Dummy_viaPoint_table'});

d_handover = VrepAgent(vrepObj, 'humanFinalHandoverPosition');
d_handover.getDummyHandlesFromVREP({'handoverPosition'});


% load lookup table
lookupTraj1 = load_tables('lookupTraj1');
lookupTraj2 = load_tables('lookupTraj2');


%% Load and update scene with 

default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location
    
if 1 % This is a placeholder for the function that reads the ROS poses    
    placeHolderParam.viaPoint.stdPos = [-0.1  0.1  -0.2];    % meters
    placeHolderParam.viaPoint.stdRot = d2r([0  0  10]); % radians world coordinates
    
    placeHolderParam.handOver.stdPos = [0.0  0.30 -0.20];
    placeHolderParam.handOver.stdRot = d2r([ -30  0  10]);
    
    placeHolderParam.deterministic = 1; % make sure the shift is exact. Otherwise use it as std noise.
    
    [posesFromROS, tmpvp, tmpreba] = placeholder_get_positions(d_viaPoint, d_handover, placeHolderParam);
else
    load([storePath '/posesFromROS.txt']);
end

fprintf('Poses from ROS received\n');
% fix quaternion representation from Matlab to ROS
posesFromROS = changeQuaternionOrder(posesFromROS);

if 0 % This is to avoid updating the via point trajectory
    posesFromROS(1,:) = -999*ones(1,7);
end


if  sum(posesFromROS(1,:)-(-999*ones(1,7)))==0 % execute via point update
    viaPointUpdateRequired = false;
else
    viaPointUpdateRequired = true;
    viaPoint.T = fromQuaternionToHomog( posesFromROS(1,:) );
    d_viaPoint.sendTargetCartesianCoordinates(viaPoint.T(1:3,4), tr2rpy(viaPoint.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);    
end

if  sum(posesFromROS(2,:)-(-999*ones(1,7)))==0 % execute via point update
    handOverUpdateRequired = false;
else
    handOverUpdateRequired = true;
    rebaHand.T = fromQuaternionToHomog(posesFromROS(2,:));    
    d_handover.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), d_handover.getHandle('handoverPosition'), 1);
    humanA.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), humanA.getHandle('Dummy_tip_humanA_R'), 1);    
end


%% get ROS poses and update the scene in VREP

if viaPointUpdateRequired   

    traj1initGuess = search_lookupTable(lookupTraj1, viaPoint.T);
    
    % add grasping approach. 
    T_appr1.T = move_XYZ_on_intrinsic_frame(viaPoint.T, traj1initGuess.vpAppr.xyzShift);
    
    traj1dmpConnect = connect_with_DMP_wrapper(robot, [], robot.TrestPosture, T_appr1.T, traj1initGuess.sol.q);

    % reach the last state by a straight trajectory
    addEnd = robot.goTo(traj1dmpConnect.T(:,:,end), viaPoint.T, 20);
    traj1dmpWithGrasp.T            = cat(3, traj1dmpConnect.T, addEnd);

    d_viaPoint.sendTargetCartesianCoordinates(viaPoint.T(1:3,4), tr2rpy(viaPoint.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);    
    if paramGeneral.debugMode    
        robot.backToRestPosture();    
        replay_solution(traj1dmpWithGrasp.T, robot, [], [], 0, 1); 
    end    
    save('currentViaPointSolution.mat', 'T_appr1', 'traj1dmpWithGrasp');
else
    load('currentViaPointSolution.mat');
end

if handOverUpdateRequired 
    
    % find the closes pre-computed solution in terms of the final position
    % of the trajectory
    traj2initGuess = search_lookupTable(lookupTraj2, rebaHand.T);
    
    % add grasping approach. 
    T_appr2.T = rebaHand.T;
    T_appr2.T = T_appr2.T*my_trotx(pi); % mirror
    T_appr2.T = move_XYZ_on_intrinsic_frame(T_appr2.T, [0 0 -0.2]');
    d_viaPoint.sendTargetCartesianCoordinates(T_appr2.T(1:3,4), tr2rpy(T_appr2.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);

    traj2dmpConnect = connect_with_DMP_wrapper(robot, [], T_appr1.T, T_appr2.T, traj2initGuess.sol.q);
    
    % add the start part that removes the object from its current position
    removeObject = robot.goTo(traj1dmpWithGrasp.T(:,:,end), traj2dmpConnect.T(:,:,1), 20);
    traj2dmpWithGrasp.T = cat(3, removeObject, traj2dmpConnect.T);

    if paramGeneral.debugMode    
        robot.sendTargetCartesianCoordinates(traj2dmpWithGrasp.T(1:3,4,1), tr2rpy(traj2dmpWithGrasp.T(:,:,1)), robot.getHandle('Dummy_target'), 1)
        replay_solution(traj2dmpWithGrasp.T, robot, [], [], 0.01, 1); 
    end

end


%% Prepare solution to be sent

[traj1, traj2] = prepare_solution_for_real_experiment(robot, traj1dmpWithGrasp, traj2dmpWithGrasp);

tFinal  = [8  8];
paramGeneral.nTraj = 150; 
paramGeneral.filterOrder = 3; 
paramGeneral.filterFreq = 0.1;
paramGeneral.plotFlag = 0;
traj1 = add_time_and_velocity(traj1, tFinal(1), paramGeneral);
traj2 = add_time_and_velocity(traj2, tFinal(2), paramGeneral);


robot.simStop;
% do FK and also recover the homog. transf. matrix
for k=1:numel(traj1.q(:,1))
    robot.setJointAngles(traj1.q(k,:));
end

for k=1:numel(traj2.q(:,1))
    robot.setJointAngles(traj2.q(k,:));
end     
robot.simStart;

































