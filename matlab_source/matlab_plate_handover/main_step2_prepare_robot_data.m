
    

clear; clc; close all; dbstop if error;
initialize_dropbox_path(1, 1);
                   
%% Initialize vrep to run FK and visual inspection in VREP
vrepObj = Vrep( );

% Initialize human demonstrator A (the human that will be improved by REBA)
humanA = VrepAgent(vrepObj, 'human');
humanA.getDummyHandlesFromVREP({'Dummy_tip_humanA_R'});

robot = VrepAgent(vrepObj, 'robot');
robot.simStart;
robot.requestStreaming; % this is important!
robot.getDummyHandlesFromVREP({'Dummy_target', 'Dummy_tip'});
robot.getJointHandles({'Baxter_rightArm_joint1', 'Baxter_rightArm_joint2', 'Baxter_rightArm_joint3', 'Baxter_rightArm_joint4', ...
                        'Baxter_rightArm_joint5', 'Baxter_rightArm_joint6', 'Baxter_rightArm_joint7'});



%%
paramGeneral.nTraj = 150;
paramGeneral.filterOrder = 3;
paramGeneral.filterFreq = 0.1;


%% 
fileName = '20151111_001941_grasp_plate_on_table.mat';
load(fileName);
% post process for running in the solution in the real Darias
[traj1, traj2] = prepare_solution_for_real_experiment(robot, solutionRobotFinal1, solutionRobotFinal2);


tFinal  = [8  8];
traj1 = add_time_and_velocity(traj1, tFinal(1), paramGeneral);
traj2 = add_time_and_velocity(traj2, tFinal(2), paramGeneral);



keyboard
save([fileName(1:end-4) '_ROS.mat'], 'traj1', 'traj2' );

break

%%
robot.simStop;
% do FK and also recover the homog. transf. matrix
for k=1:numel(traj1.q(:,1))
    robot.setJointAngles(traj1.q(k,:));
end
pause(1);
for k=1:numel(traj2.q(:,1))
    robot.setJointAngles(traj2.q(k,:));
end     


%






