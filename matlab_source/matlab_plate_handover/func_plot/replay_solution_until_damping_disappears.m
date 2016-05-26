function [Trobot, qrobot, TrobotActual] = replay_solution_until_damping_disappears(sol, robot)
% 

    %% This part is very important to guarantee smooth starts!!!
    robot.TrestPosture = sol(:,:,1);
    robot.sendTargetCartesianCoordinates(robot.TrestPosture(1:3,4), tr2rpy(robot.TrestPosture), robot.getHandle('Dummy_target'), 1);
    [~, ~, ~, robot.qRestPosture] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
    robot.backToRestPosture;
    disp('waiting for arm to stabilize at rest posture');
    pause(1.50); 
    
    %% code
    indexDummyTip    = robot.getIndex('Dummy_tip');
    handleDummyRobot = robot.getHandle('Dummy_target');    
    
    nExtrasStart = 5;
    nExtrasEnd   = 5;    
    
    % add some extra frames at the beginning
    for j = 1:nExtrasStart
        sol = cat(3, sol(:,:,1), sol);
    end
        
    % add some extra frames at the end
    for j = 1:nExtrasEnd
        sol = cat(3, sol, sol(:,:,end));
    end    
    
    for k=1:numel(sol(1,1,:))        
        Trobot(:,:,k) = sol(:,:,k);
        robot.sendTargetCartesianCoordinates(sol(1:3,4,k) ,...
                                             tr2rpy( sol(:,:,k) ), handleDummyRobot, 1);

        % read joint trajectories
        [~, ~, TrobotActual(:,:,k), qrobot(k,:)] = robot.readGenericCoordinates(indexDummyTip);
    end

end