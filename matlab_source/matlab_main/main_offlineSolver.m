
    
clear; clc; close all; dbstop if error;
add_required_paths();

if 1 % table
    typeOfDemoFolderName = 'interaction_data_plate_on_table';   
    keyName{1} = 'table1_20151111_113045';
    keyName{2} = 'table2_20151111_114051';
    keyName{1} = 'table1_20151130_121146';
    keyName{2} = 'table2_20151130_115714';    
end

REBAposeNum = []; % the number represents the pose in the table entry (getREBAPose.m)
                  % if empty the fake reba pose generator will be used.

% Tune parameters
demo.fileLocation = ['../demonstration_code_and_postprocessing/' typeOfDemoFolderName '/post_processed_data/set1.mat'];
demo.typeOfCollaboration = 'grasp plate on table';
nUpdates  = 70;
nRollOuts = 25;

ikCost    = inf;
initGuessOnPolicy = []; % if you add the name of a previous solution here the policy will be reload from there
                        % this can speed up the solution.            
% Initialize vrep object.
vrepObj = Vrep( );

% Initialize human demonstrator A (the human that will be improved by REBA)
humanA = VrepAgent(vrepObj, 'human');
humanA.getDummyHandlesFromVREP({'Dummy_tip_humanA_R'});

% Initialize human demonstrator B (the trajectory that Darias will use)
humanB = VrepAgent(vrepObj, 'human');
humanB.getDummyHandlesFromVREP({'Dummy_tip_humanB_R'});

% Initialize Baxter
robot = VrepAgent(vrepObj, 'robot');
robot.getDummyHandlesFromVREP({'Dummy_target', 'Dummy_tip', 'Dummy_tipFK'});

robot.requestStreaming; % this is important!

robot.opt.floorHeight = -1.220; % meters
robot.opt.tableHeight = -0.475; % meters                    

% Tool, object or via point
% =======================================
switch typeOfDemoFolderName
    case 'interaction_data_plate_on_table'
        viaPointDummyName = 'Dummy_viaPoint_table';
    case 'interaction_data_plate_on_shelf'
        viaPointDummyName = 'Dummy_viaPoint_shelf';
end
d_viaPoint = VrepAgent(vrepObj, 'viaPoint'); 
d_viaPoint.getDummyHandlesFromVREP({viaPointDummyName});
[~,~, toolPos.T] = d_viaPoint.readGenericCoordinates(d_viaPoint.getIndex(viaPointDummyName));        
clear agentViaPoint;
d_handover = VrepAgent(vrepObj, 'humanFinalHandoverPosition');
d_handover.getDummyHandlesFromVREP({'handoverPosition'});
[~,~, humanA.Treba] = d_handover.readGenericCoordinates(d_handover.getIndex('handoverPosition'));

default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location

%% Shake the positions to optimize new trajectories
if 1
    scale = [0 0] ;
    placeHolderParam.viaPoint.stdPos = scale(1)*[-0.1 0.1  -0.12];    % meters
    placeHolderParam.viaPoint.stdRot = scale(1)*d2r([10  30  -50]); % radians world coordinates
    placeHolderParam.handOver.stdPos = scale(2)*[-0.2  0.2  0];
    placeHolderParam.handOver.stdRot = scale(2)*d2r([ 0 0  0]);
    placeHolderParam.deterministic = 1; % make sure the shift is exact. Otherwise use it as std noise.
    [posesFromROS, tmpvp, tmpreba] = placeholder_get_positions(d_viaPoint, d_handover, placeHolderParam);

    % Get real reba pose
    if ~isempty(REBAposeNum)
        posesFromROS(2,:) = getREBAPose(REBAposeNum);
    end
    posesMatlabFormat = changeQuaternionOrder(posesFromROS);

    toolPos.T = fromQuaternionToHomog( posesMatlabFormat(1,:) );
    d_viaPoint.sendTargetCartesianCoordinates(toolPos.T(1:3,4), tr2rpy(toolPos.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);    

    humanA.Treba = fromQuaternionToHomog(posesMatlabFormat(2,:));    
    d_handover.sendTargetCartesianCoordinates(humanA.Treba(1:3,4), tr2rpy(humanA.Treba), d_handover.getHandle('handoverPosition'), 1);
    humanA.sendTargetCartesianCoordinates(humanA.Treba(1:3,4), tr2rpy(humanA.Treba), humanA.getHandle('Dummy_tip_humanA_R'), 1); 
end



%%

% get current position as rest posture
if 0
    [~, ~, restT, qRest] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
    save('../../config/baxterRest.mat', 'restT', 'qRest');
else
    load('../../config/baxterRest.mat');
end
robot.qRestPosture = qRest; % robot.setJointAngles( qRest );
robot.TrestPosture = restT; 
robot.backToRestPosture();
robot.sendTargetCartesianCoordinates(restT(1:3,4), tr2rpy(restT), robot.getHandle('Dummy_target'), 1); 


% Initialize human
human = VrepAgent(vrepObj, 'human');
human.getDummyHandlesFromVREP( {'human_head'});


% load natural demonstrations and return it in two parts, before and after grasping
[demo, viaPoint, hworkspc]   = load_interaction(demo, toolPos.T, humanA.Treba);
param.hfig.hworkspc = hworkspc;
view([1 -1 2]);
set_fig_position([0.496 0.05 0.507 0.95]);
%xlim([0.0 1.5])

% add grasping approach. 
[demo.part{1}.vpAppr, demo.part{1}.vpGrasp] = grasping_approach_as_via_point(viaPoint.part{1}, [0 0 -0.1]', 30);

% for the hand over the approach and grasp are the same
[demo.part{2}.vpAppr, demo.part{2}.vpGrasp] = grasping_approach_as_via_point(viaPoint.part{2}, [0 0 0]', 0);

% Visualize the hands moving in space during the natural demonstration
% ========================================
if 0
    xyz = demo.part{2}.human.T(1:3,4);
    rpy = tr2rpy(demo.part{2}.human.T);
    humanA.sendTargetCartesianCoordinates(xyz, rpy, humanA.getHandle('Dummy_tip_humanA_R'), 1);

    for p=1:2;
        for k=1:numel(demo.part{p}.robot.T(1,1,:));
            xyz = demo.part{p}.robot.T(1:3,4,k);
            rpy = tr2rpy(demo.part{p}.robot.T(:,:,k));
            humanB.sendTargetCartesianCoordinates(xyz, rpy, humanB.getHandle('Dummy_tip_humanB_R'), 1);
            pause(0.01)
        end
    end
end


if 0 % check via points are actually feasible 
    indexDummyTip    = robot.getIndex('Dummy_tip');
    handleDummyRobot = robot.getHandle('Dummy_target');
    
    for p=1:2
        robot.sendTargetCartesianCoordinates(demo.part{p}.vpGrasp.T(1:3,4), tr2rpy( demo.part{p}.vpGrasp.T), handleDummyRobot, 1);
        pause(1); % wait for damped least square
        [~, ~, TrobotActual] = robot.readGenericCoordinates(indexDummyTip); 
        fprintf('Error xyz %g %g %g \n', TrobotActual(1:3,4)-demo.part{p}.vpGrasp.T(1:3,4,1));
        fprintf('Error rpy %g %g %g (deg)\n', r2d(tr2rpy( demo.part{p}.vpGrasp.T(:,:,1) )-tr2rpy(TrobotActual)) );        
        pause;
    end  
end

%% Load general parameters
% =======================================
param.ikine    = userParamIKine(ikCost);
param.reps     = userParamREPS(nUpdates, nRollOuts);
param.plot     = userParamPlot(param.reps.nUpdates);
param.repsCost = userParamCostREPS();
param.ViaPointStopError = 2.5; % cm average error during via point


%% part 1
p = 1;
% Compute initial guess by providing the initial translation and rotation of the trajectories
% ===========================
xyTranslationOffset = demo.part{p}.vpAppr.T(1:3,4)-demo.part{p}.robot.p(:,end);

% Find the rotation angle 
h_ = [];%  figurewe('debugAngles'); xlabel 'x'; ylabel 'y'; zlabel 'z';
angles1 = homogTransfMatrixProjectedAngles(demo.part{p}.vpAppr.T(:,:,1), h_ );
angles2 = homogTransfMatrixProjectedAngles(demo.part{p}.robot.T(:,:,end), h_ );
deltaRPY = wrap2pi(angles1.vectorY.aroundWorldXYZ(3))-wrap2pi(angles2.vectorY.aroundWorldXYZ(3));
zRotationOffset = wrap2pi(deltaRPY);
centerRotation  = demo.part{p}.vpAppr.T(1:3,4);    

figure(hworkspc); 
[init, frameShift] = initial_guess(robot.TrestPosture, demo.part{p}, xyTranslationOffset, zRotationOffset, centerRotation, hworkspc, 0);
param.frameRotateCenter = frameShift.dynamic.zRotPivotMoving;
  
% This becomes dependent on the part
robot.initialCartesianTrajectory = init.robot;
human.initialCartesianTrajectory = init.human;

% Check deviations
param.hfig.auxDeviation = figurewe('hfigAuxDeviation');
set_fig_position([0.122 0.0704 0.293 0.886]);
subplot(2,1,1); hold on; grid on; ylabel 'xy'; axis 'equal';
subplot(2,1,2); hold on; grid on; ylabel 'yz'; axis 'equal';


[traj1Raw, ~, keyName1] = trajectory_optimization(robot, demo.part{p}, param, init, hworkspc, keyName{1});

% force solution to have close to zero velocity at the start, by adding
% some extra steps and smoothing the trajectory
%traj1RawS = force_smooth_raw_start(traj1Raw, 5);

% Post process part 1
paramFilterJoint.active=1;
traj1dmpSmoothStart = connect_with_DMP_wrapper(robot, traj1Raw.T, robot.TrestPosture, [], paramFilterJoint);

% reach the last state by a straight trajectory
addEnd = robot.goTo(traj1dmpSmoothStart.T(:,:,end), demo.part{1}.vpGrasp.T, demo.part{1}.vpAppr.nSteps);
traj1dmpWithGrasp.T  = cat(3, traj1dmpSmoothStart.T, addEnd);

robot.backToRestPosture();                     
replay_solution(traj1dmpWithGrasp.T, robot, [], [], 0.02, 1); 

solTraj1.sol     = traj1dmpSmoothStart;
solTraj1.vpAppr  = demo.part{p}.vpAppr;
solTraj1.vpGrasp = demo.part{p}.vpGrasp;
save(['./lookupTraj1/' keyName1 '.mat'], 'solTraj1');


keyboard

%% part 2
p = 2;

% move human hand to handover position
humanA.sendTargetCartesianCoordinates(humanA.Treba(1:3,4), tr2rpy(humanA.Treba), humanA.getHandle('Dummy_tip_humanA_R'), 1);

% set the rest posture as the beginning of the grasp approach
robot.TrestPosture = demo.part{1}.vpAppr.T;
robot.sendTargetCartesianCoordinates(robot.TrestPosture(1:3,4), tr2rpy(robot.TrestPosture), robot.getHandle('Dummy_target'), 1);
[~, ~, ~, robot.qRestPosture] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
robot.backToRestPosture;

% Compute initial guess by providing the initial translation and rotation of the trajectories
% ===========================
xyTranslationOffset = demo.part{p}.vpAppr.T(1:3,4)-demo.part{p}.robot.p(:,end);

% Find the rotation angle 
h_ = [];%  figurewe('debugAngles'); xlabel 'x'; ylabel 'y'; zlabel 'z';
angles1 = homogTransfMatrixProjectedAngles(demo.part{p}.vpAppr.T, h_ );
angles2 = homogTransfMatrixProjectedAngles(demo.part{p}.robot.T(:,:,end), h_ );
deltaRPY = wrap2pi(angles1.vectorZ.aroundWorldXYZ(3))-wrap2pi(angles2.vectorZ.aroundWorldXYZ(3));
zRotationOffset = wrap2pi(deltaRPY);
centerRotation  = demo.part{p}.vpAppr.T(1:3,4);    

figure(hworkspc); 
[init, frameShift] = initial_guess(robot.TrestPosture, demo.part{p}, xyTranslationOffset, zRotationOffset, centerRotation, hworkspc, 0);
param.frameRotateCenter = frameShift.dynamic.zRotPivotMoving;

% This becomes dependent on the part
robot.initialCartesianTrajectory = init.robot;
human.initialCartesianTrajectory = init.human;

[traj2Raw, ~, keyName2] = trajectory_optimization(robot, demo.part{p}, param, init, hworkspc, keyName{2});
        
% Post process
paramFilterJoint.active=1;
traj2dmpSmoothStart = connect_with_DMP_wrapper(robot, traj2Raw.T, robot.TrestPosture, [], paramFilterJoint);

% add the start part that removes the object from its current position
removeObject = robot.goTo(demo.part{1}.vpGrasp.T, traj2dmpSmoothStart.T(:,:,1), demo.part{1}.vpAppr.nSteps);
traj2dmpWithGrasp.T = cat(3, removeObject, traj2dmpSmoothStart.T);

robot.backToRestPosture;
replay_solution(traj2dmpWithGrasp.T, robot, [], [], 0.01, 1); 
                   
solTraj2.sol     = traj2dmpSmoothStart;
solTraj2.vpAppr  = demo.part{p}.vpAppr;
solTraj2.vpGrasp = demo.part{p}.vpGrasp;
save(['./lookupTraj2/' keyName2 '.mat'], 'solTraj2');





break        



% % 
% % save([my_time '_grasp_plate_on_table.mat'], 'solutionRobotFinal1', 'solutionRobotFinal2');
% % 
% % % post process for running in the solution in the real Darias
% % [traj1, traj2] = prepare_solution_for_real_experiment(robot, solutionRobotFinal1, solutionRobotFinal2);
% % 
% % 












