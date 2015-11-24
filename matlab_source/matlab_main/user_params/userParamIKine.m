function ikine = userParamIKine(ikCost)
% 
%     ikine.createInitTraj.reloadPreviousSol = 1;
%    
     
%     
%     % ====================================================
%     % This set will change the low-level solver. Avoid tuning these
%     % numbers!
%     % set default parameters for solution of inverse kinematics
%     ikine.ilimit    = 5;
%     ikine.tol       = 1e-6;
%     ikine.alpha     = 0.5;   % initial value to start learning
%    
%     ikine.varstep   = true;    % variable step
%     ikine.alpha_max = 0.5;   % gjm: saturate max rate to improve
%     ikine.alpha_min = 0.001; % gjm: saturate min rate to improve
%     % ==========================================================
%     
%     
%     ikine.plot        = false;  % this will plot some values useful to debug
% 
%     ikine.pinv          = 1; % pinv is usually much better than transpose
%     ikine.pinv_restPost = 0; % pinv is usually much better than transpose
% 
%     ikine.verbose = 0;
%  
%     ikine.pInvNoise = 0.01;   
%     
%     ikine.connectTraj.maxSteps = 10; % The maximum number of steps that the trajectory going from
%                                           % the arm rest posture to the beginning of the trajectory will
%                                           % have

    ikine.createInitTraj.nSamples = 10;
    
    ikine.IKErrorCost = 100;
    
    % this is used to resample the IK error. It is legacy from the old method I was using to compute
    % the IK from my own IK implementation.
    ikine.nResampleError = ikine.createInitTraj.nSamples + 20;
    
    ikine.minCost = ikCost; % criterion to stop ikne solution in the outer loop
    
    
    ikine.QrestPosture = 10; % how much weight to put so that the arm joint trajectories
                            % stay close to the initial position.
                            
    ikine.Qodometry = 1000; % penalize for joint movement
    
    ikine.mask = logical([1 1 1    1 1 1]);
    
    
    if 1
        ikine.QrestPosture = 0;
        ikine.Qodometry = 0;
    end
    
end





