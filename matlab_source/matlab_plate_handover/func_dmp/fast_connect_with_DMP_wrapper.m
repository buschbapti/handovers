function [trajDMP] =  fast_connect_with_DMP_wrapper(robot, qTraj, TorQstart, Tend, dmpExtendTimeFactor)
% [trajdmpApproach] =  fast_connect_with_DMP_wrapper(robot, qTraj, TorQstart, Tend, dmpExtendTimeFactor)
% use DMPS to force start and goal states
%
%
%
% INPUT
%   paramFilterJoint.active: 1 or 0. If active, lowpass filter the joint to avoid unexpected jumps of DMP
%   paramFilterJointplotDebug: optional field. 1 or 0. Check the result of smoothing

    
    if ~isempty(Tend)
        % do IK at the ideal goal state    
        robot.sendTargetCartesianCoordinates( Tend(1:3,4), tr2rpy(Tend), robot.getHandle('Dummy_target'), 1);
        [~, ~, ~, qEnd] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
    else
        qEnd = [];
    end
    
    if ~isempty(TorQstart)
        if sum( size(TorQstart) == [4 4]) ==2 % You sent the homog transf. matrix
            % do IK at the ideal initial state    
            robot.sendTargetCartesianCoordinates( TorQstart(1:3,4), tr2rpy(TorQstart), robot.getHandle('Dummy_target'), 1);
            pause(0.2); % it is better to wait a little here such that the IK can stabilize the dumping
            [~, ~, ~, qStart] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
        end
        if numel(TorQstart) == 7 % you sent joint coordinates
            qStart = TorQstart;
        end
    else
        qStart = [];
    end
    

    % Do the DMP
    if exist('dmpExtendTimeFactor', 'var')
        trajDMP = connect_with_DMP_fast(robot, qStart, qEnd, qTraj,  dmpExtendTimeFactor );
    else
        trajDMP = connect_with_DMP_fast(robot, qStart, qEnd, qTraj);
    end    
    
end

function solutionRobotFinal2 = connect_with_DMP_fast(robot, qIdealStart, qIdealFinal, qOld,  dmpExtendTimeFactor)
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
    param.alphaBetaFactor          = 4; % smaller values gives less damping 4 but 6 maybe good
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
    
    % Do a joint check limit here. Got these values from VREP
    j=1;
    baxter.joint(j).min   = -9.750e+01;
    baxter.joint(j).range = 1.950e+02;    
    
    j=2;
    baxter.joint(j).min   = -1.230e+02;
    baxter.joint(j).range = 1.830e+02; 

    j=3;
    baxter.joint(j).min   = -1.750e+02;
    baxter.joint(j).range = 3.500e+02;
    
    j=4;
    baxter.joint(j).min   = -2.865e+00;
    baxter.joint(j).range = 1.529e+02;

    j=5;
    baxter.joint(j).min   = -1.753e+02;
    baxter.joint(j).range =3.505e+02;
    
    j=6;
    baxter.joint(j).min   = -9.000e+01;
    baxter.joint(j).range = 2.100e+02;

    j=7;
    baxter.joint(j).min   = -1.753e+02;
    baxter.joint(j).range = 3.505e+02;
    
    for j=1:7 % saturate joint limits here
        trajOut(:,j) = saturate( trajOut(:,j), d2r(baxter.joint(j).min), d2r(baxter.joint(j).min+baxter.joint(j).range) );
    end
    
    
    
    solutionRobotFinal2.T = [];
    solutionRobotFinal2.p = [];
    solutionRobotFinal2.q = trajOut;    
    
end

function q = saturate( q, minr, maxr )

    % saturate low
    idxMin = q < minr;
    q(idxMin) = minr;
    
    % saturate high
    idxMax = q > maxr;
    q(idxMax) = maxr;

    
end








