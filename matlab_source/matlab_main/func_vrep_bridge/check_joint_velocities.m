
function  check_joint_velocities(vrepTraj)


    % Optimize start
    % =====================
    % search for the joint with the highest initial velocity    
    [val, dof] =  max( abs(vrepTraj.q(1,:)) );
    y = vrepTraj.q(:,dof);
    t = 0:vrepTraj.dt:(numel(y)-1)*vrepTraj.dt;

    figurew('worstJoint');
    plot(t(1:end-1), diff(y)./diff(t)', sty('b', 'o', 2, [], 5)  );
  
    figurew('allJoints');
    qd = bsxfun(@rdivide, diff(vrepTraj.q), diff(t)');
    plot(t(1:end-1), qd, 'LineWidth', 5  );
    
    
end














