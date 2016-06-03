function robotData = execute_real_robot_screw_wrap(calibShelfTraj, timeGoBack, sl_ip, soundFiles)

    dmpSolution.q  = calibShelfTraj.qgo;
    dmpSolution.dt = timeGoBack(1)/numel(dmpSolution.q(:,1));

    returnFromShelf.q  = calibShelfTraj.qdeliver;
    returnFromShelf.dt = timeGoBack(1)/numel(returnFromShelf.q(:,1));


    graspScrew.q = calibShelfTraj.grasp.q;
    graspScrew.dt = timeGoBack(3)/numel(graspScrew.q(:,1));

    graspReturn = graspScrew;
    graspReturn.q = graspReturn.q(end:-1:1,:);

    % send trajectory to Darias
    robotData = execute_real_robot_screw(sl_ip, soundFiles, dmpSolution, returnFromShelf, graspScrew, graspReturn );
    
end