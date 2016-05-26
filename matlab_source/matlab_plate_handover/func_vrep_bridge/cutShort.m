function traj =  cutShort(traj, nSize)
% Decrease the last points of the trajectory by nSize


    traj.p = traj.p(:,1:end-nSize);
    traj.T = traj.T(:,:,1:end-nSize);
    traj.q = traj.q(1:end-nSize,:);



end