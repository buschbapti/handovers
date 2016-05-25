
if 0
    initialize_dropbox_path(1,0,1);
    addpath('./func_robot_vrep');
    addpath('./func_robot_vrep/func_vrep_bridge');
    disp('** set ik damping to 0.1 in vrep**')
    disp('** IK weight resolution linear = 1, angular =1 **')
    pause();
end

clear; clc; close all; dbstop if error;
robot = initialize_vrep_baxter('elbow_down');




for k =  1:4
    clear out

    %out = grid_endeffector_positions_insertion_toy('grid_relative', k);
    out = grid_endeffector_positions_insertion_toy('grid_reba', k);
    
    out.qball = getBallPoses(out.q);
    robot.sendTargetCartesianCoordinates(out.Tinria(1:3,4), tr2rpy(out.Tinria), robot.getHandle('Dummy_target'), 1)
    robot.setJointAngles(out.q,1);
    robot.setJointAngles(out.q);
    
    for kk = 1:5
        qstep = out.qball(kk,:);
        if kk>1
            animateTransition(robot, out.qball(kk-1,:), qstep, 20 );
            pause(0.5)
        else
            robot.setJointAngles(qstep,1);
            pause(1)
        end
    end
    
end




