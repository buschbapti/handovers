function [calib] = calibrate_shelf(robot, Tnow, qnow,  adjust)


    calib.grasp = adjust.fineCalibration;
    robot.sendTargetCartesianCoordinates(calib.grasp.T(1:3,4), tr2rpy(calib.grasp.T), robot.getHandle('Dummy_target'), 1);
    robot.setJointAngles(qnow);
    calib.grasp.q = robot.getJointAngles();
    
    % keep the dummy there as a reference
    robot.sendTargetCartesianCoordinates(calib.grasp.T(1:3,4), tr2rpy(calib.grasp.T), robot.getHandle('tool'), 1);
    
    % approach position    
    calib.approach.T =  calib.grasp.T*se3(0, 0, 0, 0, 0, adjust.approach_Z_hand);
    calib.approach.T =  se3(0, 0, 0, 0, 0, adjust.approach_Z_world)*calib.approach.T;
    robot.sendTargetCartesianCoordinates(calib.approach.T(1:3,4), tr2rpy(calib.approach.T), robot.getHandle('Dummy_target'), 1);
    calib.approach.q= robot.getJointAngles();
    
    % create trajectory in joint space
    for j=1:7
        calib.graspTraj.q(:,j) = goToTrapez(calib.approach.q(j), calib.grasp.q(j), adjust.approach_nTraj, []);
    end
    
% % % %     
% % % %     
% % % %     if ~isempty(adjust.go.T)
% % % %         calib.Tgo        = adjust.go.T; % new position is given as a full homog. transf. matrix
% % % %     else
% % % %         calib.Tgo        = Tnow;
% % % %     end
% % % %     
% % % %     if ~isempty(adjust.go.se3) % add some explicit perturbation as se3 parameters
% % % %         calib.Tgo = calib.Tgo*se3(adjust.go.se3(1), adjust.go.se3(2), adjust.go.se3(3), adjust.go.se3(4), adjust.go.se3(5), adjust.go.se3(6));
% % % %     else % create some random stuff do debug
% % % %         calib.Tgo(1:3,4) = 0.05*randn(3,1)+calib.Tgo(1:3,4);
% % % %     end
% % % %     
% % % %     % find final position of joints 
% % % %     % 1st: set the arm to the closest shelf and force joints
% % % %     robot.sendTargetCartesianCoordinates(Tnow(1:3,4), tr2rpy(Tnow), robot.getHandle('Dummy_target'), 1);
% % % %     robot.setJointAngles(qnow);
% % % %     
% % % %     % 2nd:set the arm to the perturbed position
% % % %     robot.sendTargetCartesianCoordinates(calib.Tgo(1:3,4), tr2rpy(calib.Tgo), robot.getHandle('Dummy_target'), 1);
% % % %     % 3rd: read the joint angles as targets
% % % %     calib.qgo = robot.getJointAngles();
% % % % 
% % % %     % keep the dummy there as a reference
% % % %     robot.sendTargetCartesianCoordinates(calib.Tgo(1:3,4), tr2rpy(calib.Tgo), robot.getHandle('tool'), 1);
% % % %     
% % % %     % send arm to grasp 
% % % %     calib.grasp.T = calib.Tgo*se3(adjust.grasp.se3(1), adjust.grasp.se3(2), adjust.grasp.se3(3), adjust.grasp.se3(4), adjust.grasp.se3(5), adjust.grasp.se3(6)  );
% % % %     dz = calib.Tgo(3,4)-calib.grasp.T(3,4);
% % % %     calib.grasp.T = se3(0, 0, 0, 0, 0, dz)*calib.grasp.T;
% % % %     robot.sendTargetCartesianCoordinates(calib.grasp.T(1:3,4), tr2rpy(calib.grasp.T), robot.getHandle('Dummy_target'), 1);
% % % %     q_ = robot.getJointAngles();
% % % %     
% % % %     % create trajectory in joint space
% % % %     for j=1:7
% % % %         calib.grasp.q(:,j) = goToTrapez(calib.qgo(j), q_(j), adjust.grasp.nTraj, []);
% % % %     end
% % % % 
% % % %     % now find a posture where the robot pulls the plate out of the shelf
% % % %     % 1st: 
% % % %     Ta = se3(adjust.back.se3a(1), adjust.back.se3a(2), adjust.back.se3a(3), adjust.back.se3a(4), adjust.back.se3a(5), adjust.back.se3a(6));
% % % %     Tb = se3(adjust.back.se3b(1), adjust.back.se3b(2), adjust.back.se3b(3), adjust.back.se3b(4), adjust.back.se3b(5), adjust.back.se3b(6));
% % % %     calib.Tback = calib.Tgo*Ta;
% % % %     calib.Tback = Tb*calib.Tback;
% % % %     % 2nd:set the arm to the retracted position
% % % %     robot.sendTargetCartesianCoordinates(calib.Tback(1:3,4), tr2rpy(calib.Tback), robot.getHandle('Dummy_target'), 1);
% % % %     % 3rd: read the joint angles as targets
% % % %     calib.qback = robot.getJointAngles();   

end
