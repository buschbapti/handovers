
clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);
addpath('./func_robot_vrep');
addpath('./func_robot_vrep/func_vrep_bridge');
robot = initialize_vrep_baxter('elbow_down');


load('sol.mat');

if 0
    % replay solutions
    for k=1:numel(sol)

        Thandover = sol{k}.TendEff;
        robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('handoverPosition'), 1)
        q = sol{k}.q;
        for t = 1:numel(q(:,1))
            robot.setJointAngles(q(t,:), 1)
            pause(0.01)
        end

    end
end
figurew
plot(q, 'Color', [0.8 0.8 0.8], 'LineWidth', 3)



% generalize
for k=1:numel(sol)
    
    Thandover = sol{k}.TendEff;
    Thandover(1:3,4) = Thandover(1:3,4) + 1.*[0.20 0.5 0.3]'; 
    robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)
    robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('handoverPosition'), 1)
    robot.setJointAngles(sol{k}.q(end,:));
    pause(0.25); % put IK damping 0.01
    qnew = robot.getJointAngles();
    
    param.alphaBetaFactor=4;
    for j=1:7
        d{j} = dmp_regression(sol{k}.q(:,j), param);
        param.xf = qnew(j);
        qnewTraj(:,j) = dmp_generalize(d{j}, param )';
    end
    
    q = qnewTraj;
    for t = 1:numel(q(:,1))
        robot.setJointAngles(q(t,:), 1)
        pause(0.01)
    end
    
   % pause
end

figurew
plot(sol{k}.q, 'Color', [0.8 0.8 0.8], 'LineWidth', 3)
plot(qnewTraj)





k=4
paramFilter.filterOrder = 4; 
paramFilter.filterFreq = 0.05;
for j=1:7
    q(:,j) = smoothf( sol{k}.q(:,j), paramFilter );
end
figurew
plot(sol{k}.q, 'Color', [0.8 0.8 0.8], 'LineWidth', 4)
plot(q)

for t = 1:numel(q(:,1))
    robot.setJointAngles(q(t,:), 1)
    pause(0.01)
end
