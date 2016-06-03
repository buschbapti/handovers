function animateTransition(robot, q1, q2, nSteps)

    q = [];
    for j=1:7
        q = [q linspace(q1(j), q2(j), nSteps)'];
    end
    
    
    for t=1:nSteps
        robot.setJointAngles(q(t,:),1);
        pause(0.05)
    end
    
    
end