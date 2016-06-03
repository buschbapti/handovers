
if 0
    initialize_dropbox_path(1,0,1);
    addpath('./func_robot_vrep');
    addpath('./func_robot_vrep/func_vrep_bridge');
    disp('** set ik damping to 0.1 in vrep**')
    disp('** IK weight resolution linear = 1, angular =1 **')
    pause();
end


clear; clc; close all; dbstop if error;

if 1
    robot = initialize_vrep_baxter('elbow_down');
else
    robot = initialize_vrep_baxter('elbow_up');
end


type_grid = 'grid_relative';
% type_grid = 'grid_reba';

if 0 % do the optimization
    for iPos = 1:4

        fprintf('**** iPos %g\n', iPos);

        sol{iPos}{1} = [];    

        out = load_grid(type_grid, iPos);

        % get the 5 ball poses but only use the first one to compute the
        % trajectory. The remainder 4 will be found simply by rotating the last
        % joint in 360/5 steps
        out.qball = getBallPoses(out.q);

        % for each of the possible ball rotations compute a full trajectory
        for k=1:5

            close all;

            robot.setJointAngles(out.qball(k,:),1);

            % find the desired end effector target via FK
            Thandover = robot.readEntityCoordinate('Dummy_tipFK');
            robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)
            if Thandover(1,4) <= 0.75
                Thandover(1,4) = 0.75;
            end
            robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)
            robot.setJointAngles(out.qball(1,:));

            if k==1
                sol{iPos}{k} = optim_wrap(robot, Thandover, []);
            else
                sol{iPos}{k} = optim_wrap(robot, Thandover, sol{iPos}{k-1});
            end
        end
    end

    save(['./data/' type_grid '.mat'], 'sol')

else % load solutions 
    
    load(['./data/' type_grid '.mat']);
    robot.backToRestPosture

    for iPos = 2:3%numel(sol)
        
        fprintf('**** iPos %g\n', iPos);
        for k= 1:5
            
            fprintf('  *** rotation %g\n', k);
            
            s = sol{iPos}{k};
            Thandover = s.TendEffOriginal;
            robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('handoverPosition'), 1);
            robot.setJointAngles(s.q(end,:));            
            
            robot.setJointAngles(s.q(end,:), 1);
            
            for t = 1:2:numel(s.q(:,1))
                robot.setJointAngles(s.q(t,:), 1);
            end
        end
    end



end























