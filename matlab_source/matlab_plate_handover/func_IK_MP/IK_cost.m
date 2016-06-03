function [cost, neResampled] = IK_cost(robot, ik_hist, param)
    
    
    % resample ikine error as each rollout has a different number of steps
    % =====================================
    if 0
    neResampled = interp1(linspace(0,1, numel(ik_hist.ne)),ik_hist.ne, ...
                                  linspace(0,1,param.ikine.nResampleError));
    else
        neResampled = ik_hist.ne;
    end
    
                              
    cummulativeIKerror  = cumsum(neResampled.^2);
    
    if param.ikine.QrestPosture~=0
        % minimize the amount of joint movement in relation to the rest posture
        % ref = ik_hist.q(1,:);
        %ref = robot.vrepm.opt.restPosture.q;
        ref = robot.qRestPosture;
        q_shifted = bsxfun(@minus, ik_hist.q, ref);
        joint_movement = cumsum(abs(q_shifted));
        all_cummulative_joint_movement = sum(joint_movement(end,:));
    else
        all_cummulative_joint_movement =0 ;
    end
    
    if param.ikine.Qodometry~=0
        % minimize the amount of joint movement in relation to the rest posture
        % ref = ik_hist.q(1,:);
        %ref = robot.vrepm.opt.restPosture.q;
        ref = ik_hist.q(1,:);
        q_shifted = bsxfun(@minus, ik_hist.q, ref);
        joint_movement = cumsum(abs(q_shifted));
        all_cummulative_joint_odometry = sum(joint_movement(end,:));
    else
        all_cummulative_joint_odometry =0 ;
    end
    
    % ik error + distance to rest posture
    cost = param.ikine.IKErrorCost*cummulativeIKerror(end) + ...
           param.ikine.QrestPosture.*all_cummulative_joint_movement + ...
           param.ikine.Qodometry.*all_cummulative_joint_odometry;
    
    
end