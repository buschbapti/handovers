function traj = add_time_and_velocity(traj, tFinal, paramGeneral)

%     load(fileName);
%     
%     if paramTraj.segment==1
%         traj = traj1;
%     elseif paramTraj.segment==2
%         traj = traj2;
%     else
%         error('specified segment must be 1 or 2');
%     end
    
    %% Add time
    t = linspace(0, tFinal, numel(traj.q(:,1)));
    traj.t = t;

    traj.qdot = bsxfun(@rdivide, diff(traj.q)', diff(traj.t))';

    traj.dt = diff(traj.t);
    traj.dt = traj.dt(1);

    if paramGeneral.plotFlag
        check_joint_velocities(traj);
    end
    
    %% Filter data
    traj.qraw = traj.q;
    traj.q = smooth_by_filtering(traj.q, paramGeneral, paramGeneral.plotFlag);
    traj.q_readme = 'column1: shoulder joint.... column7: wrist joint';
    
    % Once I smooth it does not make sense to keep the homog transf because
    traj.T = 'run FK with smoothed trajectory';
    traj.p = 'run FK with smoothed trajectory';
    
    
end