function [demoGeneralized] = dmp(demoOr, param)
%  [demoGeneralized] = dmp(demoOr, param)
%  Generalize demonstration by DMP
%  
%
% INPUT
%
%     demoOr: is a [1  x nTrajOriginal] vector of positions
%
%     param.xi     = -2;   % initial position
%     param.xdi    = 0;    % initial velocity
%     param.xf     = 6;    % final position
%     param.nTraj  = 500;  % this is an internal parameter that resamples
%                            the trajectory before applying the DMP. The
%                            final solution is then resampled back to the 
%                            original value nTrajOriginal.
%
%     param.forcingFunctionScaling = 0; % this is usually set to zero
%     param.minVel = 0.3;  % The integration usually has 50% more time than
%                            the original demonstration to allow some time
%                            for the damping term to bring the state at the
%                            desired position. This can be, however, too
%                            long. So the code searches for the minimum
%                            velocity at the end, and cut the trajecotory
%                            at this moment.
%     
%    You do not have to fill in param completely. Only the fields
%    xi, xf are actually required.
%
%    Example1
%         param.xi     = -2;
%         param.xdi    = 0;
%         param.xf     = 6;
%         param.nTraj  = 500;
%         param.forcingFunctionScaling = 0;
%         param.minVel = 0.0;
%         param.alphaBetaFactor = 4;
%   Example2
%         param.xi     = -2;
%         param.xdi    = 0;
% 
% 
%
%
% OUTPUT
%
%     demoGeneralized is a [1  x nTrajOriginal] vector of generalized
%     positions
%
%
    
%% Fill in the entries that the user did not bother

    if ~isfield(param, 'xdi');
        param.xdi = 0; % assume initial velocity is zero
    end
    if ~isfield(param, 'forcingFunctionScaling');
        param.forcingFunctionScaling = 0; % assume no scaling based on amplitude
    end
    if ~isfield(param, 'minVel');
        param.minVel = 0.1; % assume 0.3 as a magic number
    end    
    if ~isfield(param, 'nTraj');
        param.nTraj = 500; % assume 500 as a magic number
    end    
    if ~isfield(param, 'alphaBetaFactor');
        param.alphaBetaFactor=4;
    end
    if ~isfield(param, 'timeFactorForSteadyState');
        param.timeFactorForSteadyState = 1.5;
    end
    if ~isfield(param, 'debugFigures');
        param.debugFigures = [0 0];
    end

%%    

    xi  = param.xi;  % initial condition
    xdi = param.xdi; % initial velocity condition
    xf  = param.xf;  % final condition
    if 0 % not implemented
        xdf = param.xdf; % final velocity condition
    end
    
    nTraj = param.nTraj;    
    
    % get the trajectory, resample to a reasonable number    
    tnew    = linspace(0,1,nTraj);
    tdemoOr = linspace(0,1,numel(demoOr));
    demo    = interp1(tdemoOr, demoOr, tnew);
    
    % get the derivatives
    dt = tnew(2)-tnew(1);
    demo_d  = diff(demo)/dt;
    demo_d  = [demo_d demo_d(end)];
    demo_dd = diff(demo_d)/dt;
    demo_dd = [demo_dd demo_dd(end)];
    
    movement_time = tnew(end);
    
    
    %% Initialize primitives with some heuristics
    % ====================================================
    % critically damped
    hmp.alpha_g  = 15;
    hmp.beta_g   = hmp.alpha_g/param.alphaBetaFactor;
    hmp.alpha_z  = 5;
    
    tau = 1./movement_time;
    
    n_w = nTraj/10;  % number of weights (heuristic)
    
    % initialize centers equispaced in time
    hmp.c = exp(-hmp.alpha_z*(0:1/(n_w-1):1)');
    hmp.h = (diff(hmp.c)*0.65).^2;
    hmp.h = 1./[hmp.h;hmp.h(end)];

    if param.debugFigures(1)
        haux.h2 = figurew('centers');
        subplot(2,1,1); hold on; grid on
        plot(hmp.c, SBLUEBALL ); ylabel 'center'
        subplot(2,1,2); hold on; grid on;
        plot(hmp.h, SREDBALL );  ylabel 'width'
    end

    %% Run 
    if param.forcingFunctionScaling
        hmp.A =  demo(end)-demo(1) ;
    else
        hmp.A = 1; % no scaling
    end

    target = demo(end).*ones(size(demo));
    
    % compute the transformation function values: canonical system
    Z = zeros(size(demo));  Z(1) = 1;
    for i = 2:length(demo),
        Z(i) = Z(i-1)-hmp.alpha_z*Z(i-1)*tau*dt;
    end

    if param.debugFigures(1)
        haux.h1 = figurew('h1');
        plot(tnew, demo, sty('g',[],2))
        plot(tnew, target, 'b.-')
        plot(tnew, Z, 'k--');
        fprintf('Final Z(end): %g\n', Z(end));
        legend('demo', 'target', 'canonical system');
    end

    % Following the paper
    Eq7.hi = -hmp.h*ones(1,length(demo));
    Eq7.zMinusCi = ones(length(hmp.c),1)*Z - hmp.c*ones(1,length(demo));


    % - weighting functions
    % PSI = exp( -hmp.h*ones(1,length(demo)) .* ( ( ones(length(hmp.c),1)*Z - hmp.c*ones(1,length(demo)) ).^2 ) );
    PSI = exp(  Eq7.hi.* ( Eq7.zMinusCi.^2 )  );

    
    if param.debugFigures(1) % plot PSI
        haux.h3 = createFig('PSI', 1);
        plot(tnew, PSI, SGRAY);
    end

   
    miolo1 = hmp.beta_g*(target-demo) - demo_d/tau;
    fdemo  = demo_dd/tau^2 - hmp.alpha_g*(miolo1);
    fdemo  = fdemo/hmp.A;

    % locally weighted linear regression
    %  ====================================
    miolo3 = ones(length(hmp.c),1)*( Z.^2 );
    miolo4 = ones(length(hmp.c),1)*( Z.*fdemo );
    wDnom = sum( PSI .* miolo3 ,2);
    wNom  = sum( PSI .* miolo4 ,2);
    hmp.w = wNom./(wDnom+1.e-10);   

    diag_ = diag(hmp.w);
    temp_ = diag_*PSI;
    
    if param.debugFigures(1)
        figure(haux.h1)
        createFig('temp',1 )
        plot(tnew,   temp_, 'g')
        plot(tnew,  Z.*sum(temp_,1), SBLACK(2))
    end

    %%%%%%%%%%%%%%%%%%%%%
    % run motor primitive
    %%%%%%%%%%%%%%%%%%%%%
    % time for motor primitves
    t_run = 0:dt:param.timeFactorForSteadyState*movement_time;

    % initialize motor primitive
    % - canonical system
    hmp.z = 1;
    % - initial postion and velocity
    hmp.x1      = xi;  
    hmp.x2      = xdi; 
    hmp.target  = xf;
    hmp.target_d = 0;    


    y   = demo(1);
    yd  = demo_d(1);
    ydd = demo_dd(1);


    alpha = hmp.alpha_g;
    beta  = hmp.beta_g;
    A     = hmp.A;
    tsum = 0;


    for i = 1:length(t_run)
        
        % move target
        hmp.target = hmp.target + hmp.target_d*dt;

        %if param.debugFigures(1)
        %    plot(tsum, hmp.target, 'mo');
        %end
        tsum = tsum + dt;

        % calculate weighting & transformation function
        hmp.psi = exp(-hmp.h.*((hmp.z-hmp.c).^2));
        f = sum(hmp.psi.*hmp.w*hmp.z)/sum(hmp.psi+1.e-10);

        hmp.x2_d = (alpha*(beta*(hmp.target-hmp.x1)-hmp.x2) + A*f)*tau;

        hmp.x1_d = hmp.x2*tau;

        hmp.zd = -hmp.alpha_z*hmp.z*tau;
        hmp.x1_dd = hmp.x2_d*tau;

        hmp.x2 = hmp.x2 + dt*hmp.x2_d;
        hmp.x1 = hmp.x1 + dt*hmp.x1_d;
        hmp.z = hmp.z + dt*hmp.zd;

        % store values
        y(i)   = hmp.x1;
        yd(i)  = hmp.x1_d;
        ydd(i) = hmp.x1_dd;
        
    end
    
    if 0

        % find the index where the velocity is stabilized
        for k = numel(t_run):-1:round(movement_time/dt)
            if abs( yd(k) ) > param.minVel
                break;
            end
        end
        tcut = t_run(1:k);
        ycut = y(1:k);        
    else
        tcut = t_run;
        ycut = y;
    end
    
    

    
    tRecoverOut = linspace(0,1, numel(tdemoOr));
    tRecoverIn  = linspace(0,1, numel(tcut));
    
    % resample back to original size
    demoGeneralized = interp1(tRecoverIn, ycut, tRecoverOut);
    

    if param.debugFigures(2)
        figurew; 
        subplot(1,3,1);  hold 'on'; grid on;
        plot(tnew,  demo, sty('g', [], 4));
        plot(tRecoverOut,    demoGeneralized, sty('k', [], 2));


        subplot(1,3,2);  hold 'on'; grid on;
        plot(tnew,demo_d, sty('g', [], 4));
        plot(t_run, yd, sty('k', [], 2));


        subplot(1,3,3);   hold 'on'; grid on;
        plot(tnew,demo_dd, sty('g', [], 4)  );
        plot(t_run,ydd, sty('k', [], 2)  );

    end
    
    %toc
    
end






