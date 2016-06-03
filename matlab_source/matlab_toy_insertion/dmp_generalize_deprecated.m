function ycut =  dmp_generalize(dmp, param)
%
%
% param.timeFactorForSteadyState = 1.25
%   means that the original dmp time will be extended by 25% to allow the
%   damping to stop the motion. 
%
% param.timeFactorForSteadyState = 1.0
%   means that the motion will stop at the original dmp time, no matter if
%   the desired goal was reached or not
%
% if you dont specify "timeFactorForSteadyState" the code will stop
% automatically when two conditions are satisfied:
%   1. the original dmp time has passed
%   2. the generalized dmp is close enough to the desired goal.
%   This method does not guarantee that motion will be at zero velocity
%   at the code return. But it should be slow enough that many controllers
%   wont complain.
%
%   If condition 2 cannot be respected, then it will be stopped at the
%   default value of param.timeFactorForSteadyState which is set at 2.5
%
%


    if ~isfield(param, 'xi');
        param.xi = dmp.xi;
    end
    if ~isfield(param, 'xdi');
        param.xdi = 0; % assume initial velocity is zero
    end
    if ~isfield(param, 'forcingFunctionScaling');
        param.forcingFunctionScaling = 0; % assume no scaling based on amplitude
    end
    if ~isfield(param, 'timeFactorForSteadyState');
        % whichever comes first will stop the loop
        param.timeFactorForSteadyState = 2.5; % maximum time allowed
        minError = 0.01; % minimum error to stop the integration
    else
        minError = -999;
    end

    xi  = param.xi;  % initial condition
    xdi = param.xdi; % initial velocity condition
    xf  = param.xf;  % final condition


    dt = dmp.dt;

    % time for motor primitves        
    t_run = 0:dt:param.timeFactorForSteadyState*dmp.movement_time;

    % initialize motor primitive
    % - canonical system
    hmp.z = 1;
    % - initial postion and velocity
    hmp.x1      = xi;  
    hmp.x2      = xdi; 
    hmp.target  = xf;
    hmp.target_d = 0;
    hmp.h = dmp.h;
    hmp.c = dmp.c;
    hmp.w = dmp.w;
    hmp.alpha_z = dmp.alpha_z;

    
    tau = dmp.tau;

    y   = dmp.y;
    yd  = dmp.yd;
    ydd = dmp.ydd;

    
    alpha = dmp.alpha_g;
    beta  = dmp.beta_g;
    
    if param.forcingFunctionScaling
        % not only uses the scaling, but multiplies it with 
        % param.forcingFunctionScaling if you want to manually make the
        % trajectory bigger.
        A = param.forcingFunctionScaling*abs(xf-xi); 
    else
        A = dmp.A ; 
    end
    tsum = 0;

    
    dbg=0;
    if dbg
        figurew
    end
    
    for i = 1:length(t_run)
        
        % move target
        hmp.target = hmp.target + hmp.target_d*dt;

     
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

        if dbg
            plot(i, y(i), sty('b', 'o', 2, [], 10) );        
        end
        if ( abs(hmp.target-y(i)) < minError ) && ( i > (dmp.movement_time/dt) )
            if dbg
                plot(i, hmp.target, sty('r', 'o', 2, [], 10) ); 
            end
            break
        end
        
    end
    
    tcut = t_run;
    ycut = y;

    
    %tRecoverOut = linspace(0,1, numel(tdemoOr));
    %tRecoverIn  = linspace(0,1, numel(tcut));
    
    % resample back to original size
    % demoGeneralized = interp1(tRecoverIn, ycut, tRecoverOut);
        
end




