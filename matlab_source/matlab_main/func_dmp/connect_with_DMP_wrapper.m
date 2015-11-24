function [trajDMP] =  connect_with_DMP_wrapper(robot, trajRaw, Tstart, Tend, trajRawJoint, dmpExtendTimeFactor)
% [trajdmpApproach] =  connect_with_DMP_wrapper(robot, trajRaw, Tstart, Tend)
% use DMPS to force start and goal states
    
    % 
    if exist('trajRawJoint', 'var')
        disp('joint trajectory provided for DMP generalization');
        qTraj = trajRawJoint;
    else
        % do IK to collect the current solution joint trajectories.
        qTraj = replay_solution(trajRaw, robot, [], [], 0, 1, 1); % collecting joint angles
    end
    
    if ~isempty(Tend)
        % do IK at the ideal goal state    
        robot.sendTargetCartesianCoordinates( Tend(1:3,4), tr2rpy(Tend), robot.getHandle('Dummy_target'), 1);
        [~, ~, ~, qEnd] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
    else
        qEnd = [];
    end
    
    if ~isempty(Tstart)
        % do IK at the ideal initial state    
        robot.sendTargetCartesianCoordinates( Tstart(1:3,4), tr2rpy(Tstart), robot.getHandle('Dummy_target'), 1);
        [~, ~, ~, qStart] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
    else
        qStart = [];
    end
    
    % Stop the simulation and position the current dummy target at the new
    % rest posture of the robot. This is just for visual inspection
    %robot.simStop; 
    %robot.sendTargetCartesianCoordinates(robot.TrestPosture(1:3,4) , tr2rpy(robot.TrestPosture ), robot.getHandle('Dummy_target'), 1);
    
    % Do the DMP
    if exist('dmpExtendTimeFactor', 'var')
        trajDMP = connect_with_DMP(robot, qStart, qEnd, qTraj, dmpExtendTimeFactor );
    else
        trajDMP = connect_with_DMP(robot, qStart, qEnd, qTraj);
    end
    
    
end

function solutionRobotFinal2 = connect_with_DMP(robot, qIdealStart, qIdealFinal, qOld, dmpExtendTimeFactor )
% 
% solutionRobotFinal = 
% 
%             T: [4x4x124 double]
%             p: [3x124 double]
%         theta: 'roll_out_refFrame.m bug'
%           eps: [0 0 0]
%     theta_eps: [3x1 double]


       
    % Connect gaps with DMP
    % Do an independent joint treatment    
    
    if exist('dmpExtendTimeFactor', 'var')
        param.timeFactorForSteadyState = dmpExtendTimeFactor;
    else
        param.timeFactorForSteadyState = 1.5;
    end
    param.alphaBetaFactor          = 4; % smaller values gives less damping
    param.debugFigures = [0 0];
    
    param.nTraj = 300;
    
    for j= 1:numel(qOld(1,:))
        
        trajIn = qOld(:,j)';
        
        if ~isempty(qIdealStart)
            param.xi = qIdealStart(j);
        else
            param.xi = trajIn(1);
        end
        if ~isempty(qIdealFinal)
            param.xf = qIdealFinal(j);
        else
            param.xf = trajIn(end);
        end
       
        param.minVel = 0.001;
        
        trajOut(:,j) = dmp(trajIn, param)';
        
    end 
    
    
    % Run simulation to get the forward kinematics solution    
    % Stop simulation such that FK works
    %robot.simStop; 
    disp('running IK');
    indexDummy = robot.getIndex('Dummy_tipFK');
    ctr=0;
    for  t=1:numel(trajOut(:,1))
        if ~mod(ctr,10)
            fprintf('step %g of %g.\n', t, numel(trajOut(:,1)));
        end
        robot.setJointAngles(trajOut(t,:), 1);
       
        % read again the tip, hopefully the solution makes sense
        [~, ~, T(:,:,t)]    = robot.readGenericCoordinates(indexDummy);
        ctr=ctr+1;
    end
    
    solutionRobotFinal2.T = T;
    solutionRobotFinal2.p = squeeze( T(1:3,4,:) );
    solutionRobotFinal2.q = trajOut;    
    %robot.simStart;
    
end


