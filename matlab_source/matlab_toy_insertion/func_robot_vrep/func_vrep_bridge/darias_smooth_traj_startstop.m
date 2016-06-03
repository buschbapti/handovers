function out =  darias_smooth_traj_startstop(vrepTraj, param)

    if param.reloadData == 0
        
        % Optimize start
        % =====================
        % search for the joint with the highest initial velocity    
        [val, dof] =  max( abs(vrepTraj.q(1,:)) );
        y = vrepTraj.q(:,dof);
        t = 0:vrepTraj.dt:(numel(y)-1)*vrepTraj.dt;

        % search for the best acceleration prof
        param.trajFraction = 0.015;
        param.dtTry = 0.001:0.01:0.5;
        param.minInitVel = 0.125;  
        tNew = smooth_traj_startstop(t', y, param, 'start');

        % Optimize end
        % =====================    
        % search for the joint with the highest final velocity    
        [val, dof] =  max( abs(vrepTraj.q(end,:)) );
        y = vrepTraj.q(:,dof);

        % parameters for the end ==>  ARE DIFFERENT FROM THE START PARAMETERS
        param.trajFraction = 0.10;
        param.dtTry = 0.1:0.01:1;
        param.minInitVel = 0.125;      
        tNew = smooth_traj_startstop(tNew, y, param, 'end');        
        %save('tOptimized.mat', 'tNew');        
    else
        %load('tOptimized.mat');
    end
    
    % Resample trajectory to some reasonable
    out.t = linspace(0,tNew(end), param.nTraj)';    
    out.q = interp1(tNew, vrepTraj.q, out.t);    
    out.dt = mean(diff(out.t));    
    
    dbg = 1;
    if dbg
        
        tOrig =[ 0:vrepTraj.dt:vrepTraj.dt*(numel(vrepTraj.q(:,1))-1)  ]';
        figurew('Position'); grid off;
        plot(tOrig, vrepTraj.q, 'LineWidth', 2);
        plot(out.t, out.q,  'LineWidth', 3);
        
        figurew('Velocity');
        plot(tOrig(1:end-1), bsxfun(@rdivide, diff(vrepTraj.q), diff(tOrig)), 'LineWidth', 2);
        plot(out.t(1:end-1), bsxfun(@rdivide, diff(out.q), diff(out.t)), 'LineWidth', 3);
    end
    
    % smooth with filter
    out.q = smooth_by_filtering(out.q, param);
    
end














