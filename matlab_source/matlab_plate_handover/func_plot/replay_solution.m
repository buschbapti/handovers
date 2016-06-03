function [qrobot, qhuman] = replay_solution(solutionRobot, robot, solutionHuman, human, pause_, skipFrames, skipRestPosture)
% Replay the solution for robot, and optionally for the human. To replay
% human movement you have to provide "solutionHuman" and "human", otherwise
% their input should be empty.
%
% 
%
% [qrobot, qhuman] = replay_solution(solutionRobot, robot, solutionHuman, human, pause_, skipFrames)
% [qrobot]         = replay_solution(solutionRobot, robot, [], [], pause_, skipFrames)

    if ~exist('skipRestPosture', 'var')
        robot.backToRestPosture;
    end
    
    indexDummyTip    = robot.getIndex('Dummy_tip');
    handleDummyRobot = robot.getHandle('Dummy_target');
    
    if ~isempty(human)
        handleDummyHuman = human.getHandle('human_hand_target');
    else
        qhuman = [];
    end
    
    framesToRun = 1:skipFrames:numel(solutionRobot(1,1,:));
    if framesToRun(end)~=numel(solutionRobot(1,1,:))
        framesToRun(end+1) = numel(solutionRobot(1,1,:));
    end
    
    for k = framesToRun
        
        if ~isempty(human)
            human.sendTargetCartesianCoordinates(solutionHuman(1:3,4,k) , tr2rpy( solutionHuman(:,:,k) ), handleDummyHuman, 1);
        end
        robot.sendTargetCartesianCoordinates(solutionRobot(1:3,4,k) , tr2rpy( solutionRobot(:,:,k) ), handleDummyRobot, 1);
        %robot.setJointAngles(robot.qRestPosture);
        
        % read joint trajectories
        if nargout == 1
            [~, ~, ~, qrobot(k,:)] = robot.readGenericCoordinates(indexDummyTip);
        end
        if nargout == 2
            error('TODO: reading human joint coordiantes must be implemented');
            %[~, ~, ~, qrobot(k,:)] = robot.readGenericCoordinates(indexDummyTip);
        end
        
        if isempty(pause_)
            pause;
        elseif pause_~= 0
            pause(pause_);            
        end
            
    end
    
    
end