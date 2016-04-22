function [dmp] = dmp_regression(demoOr, param)
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

    if ~isfield(param, 'xi');
        param.xi = demoOr(1); 
    end
    if ~isfield(param, 'forcingFunctionScaling');
        param.forcingFunctionScaling = 0; % assume no scaling based on amplitude
    end 
    if ~isfield(param, 'nTraj');
        param.nTraj = 200; % assume 500 as a magic number
    end    
    if ~isfield(param, 'alphaBetaFactor');
        param.alphaBetaFactor=4;
    end

%%   
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

    %% Regression
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

    % Following the paper
    Eq7.hi = -hmp.h*ones(1,length(demo));
    Eq7.zMinusCi = ones(length(hmp.c),1)*Z - hmp.c*ones(1,length(demo));

    % weighting functions
    PSI = exp(  Eq7.hi.* ( Eq7.zMinusCi.^2 )  );    
  
    miolo1 = hmp.beta_g*(target-demo) - demo_d/tau;
    fdemo  = demo_dd/tau^2 - hmp.alpha_g*(miolo1);
    fdemo  = fdemo/abs(  hmp.A  );   

    % locally weighted linear regression
    %  ====================================
    miolo3 = ones(length(hmp.c),1)*( Z.^2 );
    miolo4 = ones(length(hmp.c),1)*( Z.*fdemo );
    wDnom = sum( PSI .* miolo3 ,2);
    wNom  = sum( PSI .* miolo4 ,2);
    hmp.w = wNom./(wDnom+1.e-10);   

    diag_ = diag(hmp.w);
    temp_ = diag_*PSI;
    
    dmp.w   = hmp.w;
    %dmp.psi = hmp.psi;
    %dmp.z  = hmp.z;
    dmp.xi = param.xi;
    dmp.dt = dt;
    dmp.movement_time = movement_time;
    

    dmp.y   = demo(1);
    dmp.yd  = demo_d(1);
    dmp.ydd = demo_dd(1);

    dmp.alpha_g = hmp.alpha_g;
    dmp.beta_g = hmp.beta_g;
    dmp.A = hmp.A;
    dmp.h = hmp.h;
    dmp.c = hmp.c;
    
    dmp.tau = tau;
    dmp.alpha_z = hmp.alpha_z;
end






