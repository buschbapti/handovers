
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
            pause(0.001)
        end

    end
end



% generalize
for k=   1:numel(sol)
    
    Thandover = sol{k}.T(:,:,end);
    Thandover(1:3,4) = Thandover(1:3,4) + 0.*[-0.10 0.25 -0.23]'; 
    robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)
    %robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('handoverPosition'), 1)
    robot.setJointAngles(sol{k}.q(end,:));
    pause(0.25); % put IK damping 0.01
    qnew = robot.getJointAngles();
    
    % check desired position is achievable
    flag.e = norm(tr2delta( robot.readEntityCoordinate('Dummy_tip'), Thandover ));
    
    param=[];
    %param.Pgain = 1000;
    %param.Dgain = 100;
    for j=1:7
        d{j} = dmp_regression(sol{k}.q(:,j), param);
        param.xf = qnew(j);
        qnewTraj{j} = dmp_generalize(d{j}, param )';
    end
    q = sync_dmps(qnewTraj);
    [flag.limMin, flag.limMax] = robot.checkJointLimit(q, 1);
    
    % diagnose
    diagnose(flag.e, flag.limMin, flag.limMax);
    
    for t = 1:numel(q(:,1))
        robot.setJointAngles(q(t,:), 1)
        pause(0.0001)
    end
    
    dbg = 1;
    if dbg
        figurew
        plot(sol{k}.q, 'Color', [0.8 0.8 0.8], 'LineWidth', 3)
        plot( numel(q(:,1))*ones(7,1),  qnew', sty('b', 'o', 2, 'none', 10)    );
        plot(q)        
    end
    
    %time = linspace(0,5, );
    write_trajectory_file('./data', [linspace(0,5, numel(q(:,1)))'  q], [], 100, 2);
    % pause
end


break




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
