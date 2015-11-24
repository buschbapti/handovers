function [] = write_trajectory_file(storePath, traj1_, traj2_, nTraj)

    t1Desired = linspace(0,traj1_(end,1), nTraj);
    desiredQ = resample(traj1_(:,1), t1Desired, traj1_(:,2:end));
    traj1 = [t1Desired' desiredQ];
    
    t2Desired = linspace(0,traj2_(end,1), nTraj);
    desiredQ = resample(traj2_(:,1), t2Desired, traj2_(:,2:end));
    traj2 = [t2Desired' desiredQ];
    
    if 0
        figurew('traj1')
        plot(traj1_(:,1), traj1_(:,2:end), 'o-');
        plot(traj1(:,1),  traj1(:,2:end), 'LineWidth', 3);
        figurew('traj2')
        plot(traj2_(:,1), traj2_(:,2:end), 'o-');
        plot(traj2(:,1),  traj2(:,2:end), 'LineWidth', 3);        
    end

    try
        save([storePath '/traj1.txt'], 'traj1',  '-ascii', '-tabs');
        save([storePath '/traj2.txt'], 'traj2',  '-ascii', '-tabs');

        % save the flag. This will make ROS run again.
        empty_content = [];
        save([storePath '/flagMatlabFinished.txt'], 'empty_content',  '-ascii', '-tabs');
    catch
        warning('ROS folder structure not found.');
    end
    
end

function desiredQ = resample(originalT, desiredT, originalQ)

    desiredQ = interp1(originalT, originalQ, desiredT);

end