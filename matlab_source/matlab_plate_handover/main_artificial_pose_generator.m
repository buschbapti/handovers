
clear; clc; close all; dbstop if error;
initialize_dropbox_path(1, 1);
drawnow;

    
storePath = '../ros_matlab_bridge/src/robcom/data';

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


for k=1:10
    default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location
    
    
    placeHolderParam.viaPoint.stdPos = [0.6 0.6  1];    % meters
    placeHolderParam.viaPoint.stdRot = d2r([0  0  120]); % radians world coordinates

    placeHolderParam.handOver.stdPos = [0.6  1  1];
    placeHolderParam.handOver.stdRot = d2r([ 90  15  15]);

    placeHolderParam.deterministic = 0; % make sure the shift is exact. Otherwise use it as std noise.

    [posesFromROS, tmpvp, tmpreba] = placeholder_get_positions(d_viaPoint, d_handover, placeHolderParam);

    % fix quaternion representation from Matlab to ROS
    posesMatlabFormat = changeQuaternionOrder(posesFromROS);

    viaPoint.T = fromQuaternionToHomog( posesMatlabFormat(1,:) );
    d_viaPoint.sendTargetCartesianCoordinates(viaPoint.T(1:3,4), tr2rpy(viaPoint.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);    

    rebaHand.T = fromQuaternionToHomog( posesMatlabFormat(2,:) );    
    d_handover.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), d_handover.getHandle('handoverPosition'), 1);
    humanA.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), humanA.getHandle('Dummy_tip_humanA_R'), 1);    
    
    save( [storePath '/posesFromROS_test' num2str(k)  '.txt'], 'posesFromROS','-ascii', '-tabs');
end
















