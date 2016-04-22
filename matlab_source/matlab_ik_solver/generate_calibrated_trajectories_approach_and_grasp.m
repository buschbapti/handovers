function traj =  generate_calibrated_trajectories_approach_and_grasp(robot, pdmpA, jointDataA, pdmpB, jointDataB, calib, s, paramhandover) % IK must be OFF!

    robot.sendTargetCartesianCoordinates(calib.approach.T(1:3,4), tr2rpy(calib.approach.T), robot.getHandle('tool'), 1);
    
    % Forward traj to approach screw
    for j=1:7
        h(j) = figurew(['dmpTrajGo' num2str(j)]);
        plot( linspace(0,1,numel(pdmpA{j}.y_orig(:,1))), pdmpA{j}.y_orig  );        
        goA{j} = condition(pdmpA{j}, jointDataA{j}(1,s), calib.approach.q(j) );
        plot( linspace(0,1, numel(jointDataA{j}(:,s))), jointDataA{j}(:,s), sty('r', [], 4 ) );
        plot( linspace(0,1, numel(goA{j})), goA{j}, sty('k', [], 2 ) );
    end

    jointTraj = [goA{1}' goA{2}' goA{3}' goA{4}' goA{5}' goA{6}' goA{7}'];
    for t=1:numel(goA{1})
        robot.setJointAngles(jointTraj(t,:));
        pause(0.001);
    end
    
    traj.qgo = jointTraj; 
    
    robot.sendTargetCartesianCoordinates(calib.grasp.T(1:3,4), tr2rpy(calib.grasp.T), robot.getHandle('tool'), 1);
        
    % Forward traj to grasp screw    
    for t=1:numel(calib.graspTraj.q(:,1))
        robot.setJointAngles(calib.graspTraj.q(t,:));
        pause(0.02);
    end
    traj.grasp.q = calib.graspTraj.q;
    
    % Deliver screw on table
    for j=1:7
        h(j) = figurew(['dmpTrajGoTable' num2str(j)]);
        % plot( linspace(0,1,numel(pdmpB{j}.y_orig(:,1))), pdmpB{j}.y_orig  );  
        plot_with_exp_number( pdmpB{j}.y_orig );
        goB{j} = condition(pdmpB{j}, calib.approach.q(j), calib.deliverOnTable.q(j) );
        plot( linspace(0,1, numel(jointDataB{j}(:,s))), jointDataB{j}(:,s), sty('r', [], 4 ) );
        plot( linspace(0,1, numel(goB{j})), goB{j}, sty('k', [], 2 ) );
    end
    
    robot.setJointAngles(calib.graspTraj.q(1,:));
    
    jointTraj = [goB{1}' goB{2}' goB{3}' goB{4}' goB{5}' goB{6}' goB{7}'];
    for t=1:numel(goB{1})
        robot.setJointAngles(jointTraj(t,:));
        pause(0.001);
    end
    traj.qdeliver = jointTraj;
    
    
    %calibShelfTraj.grasp.q
end

function [] = plot_with_exp_number(y)

    t = linspace(0,1,numel(y(:,1)));
    
    nDemo = numel(y(1,:));
    for d = 1:nDemo
        plot(t, y(:,d), 'm');
        text(t(end), y(end,d), num2str(d));
        text(t(1), y(1,d), num2str(d));
    end
    
end
































