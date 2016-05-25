%
% This file is used to get the joint coordinates of Darias when getting a
% screa from the screw magazine. Then I transform this into end-effector coordinates
% in the vrep setting. This end-effector setting will be used as a goal for
% the dmp.
%
%
clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);
addpath('../func_robot_vrep');
addpath('../func_robot_vrep/func_vrep_bridge');


nTraj = 150;

%% load home position for Darias
homePosForScrewGrasping.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/home_for_screw.txt');

%% load screw data in a sequence
if 1 
    screw{1}.rawDariasData     = load('../../generic_robot_commands/screw_desired_joint_data/screw1.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw2.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw3.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw4.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw5.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw6.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw7.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw8.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw9.txt');
    screw{end+1}.rawDariasData = load('../../generic_robot_commands/screw_desired_joint_data/screw10.txt');
end

%% set vrep sign of joint angles
for k=1:10
    screw{k}.q = vrepJointFixSign(screw{k}.rawDariasData(end, 2:8));
end
homePosForScrewGrasping.q =  vrepJointFixSign(homePosForScrewGrasping.rawDariasData(end, 2:8));

%% 
h = figurewe('Cartesian'); 
set_fig_position([0.558 0.0722 0.382 0.895]);
plot_workspace(h);
dar = initialize_vrep_darias();


%% Be sure to turn IK off here!!!
for k=1:10
    fprintf('angles: %g\n');
    disp(r2d(screw{k}.q(end)))
    dar.setJointAngles(screw{k}.q);
    screw{k}.endEffPos = dar.readEntityCoordinate('Dummy_tip');
end

dar.setJointAngles(homePosForScrewGrasping.q);
homePosForScrewGrasping.endEffPos = dar.readEntityCoordinate('Dummy_tip');

break
save('../../generic_robot_commands/screw_desired_joint_data/screw_coordinates_in_vrep.mat', 'screw');
save('../../generic_robot_commands/screw_desired_joint_data/screw_coordinates_homePos_in_vrep.mat', 'homePosForScrewGrasping');











