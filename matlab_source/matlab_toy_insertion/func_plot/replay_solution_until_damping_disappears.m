function [Trobot, qrobot, TrobotActual] = replay_solution_until_damping_disappears(sol, robot)
% 

    robot.backToRestPosture;
    
    indexDummyTip    = robot.getIndex('Dummy_tip');
    handleDummyRobot = robot.getHandle('Dummy_target');    
    
    stopFlag = 0 ;    
    nTrajMax = numel(sol(1,1,:));
    
    k=1;
    while stopFlag == 0
        
        if k <= nTrajMax
            kSaturate = k;
        else
            kSaturate = nTrajMax;
        end
        
        Trobot(:,:,k) = sol(:,:,kSaturate);
        robot.sendTargetCartesianCoordinates(sol(1:3,4,kSaturate) ,...
                                             tr2rpy( sol(:,:,kSaturate) ), handleDummyRobot, 1);

        % read joint trajectories
        [~, ~, TrobotActual(:,:,k), qrobot(k,:)] = robot.readGenericCoordinates(indexDummyTip);
        
        if k >=2
            % see if the average change of joint values is less than a
            % certaion value. If so, assume the damping is finished and
            % break the loop.
            averageRateChange = sum(abs(qrobot(k-1,:)-qrobot(k,:)))/numel(qrobot(k,:));
            % disp(averageRateChange );
            if averageRateChange  < 0.001 % radians
                stopFlag=1;
            end
        end
        
        k=k+1;
        %pause(0.02);

    end
    
    
end