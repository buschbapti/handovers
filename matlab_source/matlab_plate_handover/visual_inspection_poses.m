
clear; clc; close all; dbstop if error;
add_required_paths();
drawnow;


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


default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location


mctr = 1; % main loop counter
while mctr~=41 % main loop keeps running non-stop 
    
    tic
    
    clear posesFromROS
    %  Put the scene back to its original configuration
    % ==========================================
    default_dummy_positions(robot, d_viaPoint, d_handover, 1); % return dummies to original location
    posesFromROS(1,:)   = [0.737216429048	-0.503644892162	-0.418196349287	0.0996327703676	0.994680140491	-0.0242588465096	-0.00981007382333];
    posesFromROS = [posesFromROS; reba_grid_left_right_20151211(mctr, 'right')];
 
    posesMatlabFormat = changeQuaternionOrder(posesFromROS);
    
    viaPoint.T = fromQuaternionToHomog( posesMatlabFormat(1,:) );
    d_viaPoint.sendTargetCartesianCoordinates(viaPoint.T(1:3,4), tr2rpy(viaPoint.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1); 
        
        
    if posesFromROS(3,1) == 1 % right handed
        humanHand = humanA_R;
        humanA_L.sendTargetCartesianCoordinates([1 -1 -1.2], [0 0 0],   humanA_L.getHandle('Dummy_tip_humanA_L'), 1);
        
        reba.T = fromQuaternionToHomog( posesMatlabFormat(2,:) );
        humanA_R.sendTargetCartesianCoordinates(reba.T(1:3,4), tr2rpy(reba.T), humanA_R.getHandle('Dummy_tip_humanA_R'), 1); 
    else
        humanHand = humanA_L;
        humanA_R.sendTargetCartesianCoordinates([1 -0.8 -1.2], [0 0 0], humanA_R.getHandle('Dummy_tip_humanA_R'), 1);
        
        reba.T = fromQuaternionToHomog( posesMatlabFormat(2,:) );
        humanA_L.sendTargetCartesianCoordinates(reba.T(1:3,4), tr2rpy(reba.T), humanA_L.getHandle('Dummy_tip_humanA_L'), 1);         
    end
    
    
    mctr=mctr+1;
    
    pause(1);
    
end
    
    
















    
    