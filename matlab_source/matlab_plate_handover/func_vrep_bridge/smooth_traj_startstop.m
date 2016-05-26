function [tNewFull] = smooth_traj_startstop(t0, yd, param, type)
% [tNewFull] = smooth_traj_startstop(t0, yd, param)
% Finds a new time vector such that the initial velocity starts with a
% maximum velocity value. This is useful when the trajectory to be sent to
% the robot starts with a non-zero velocity.
%
% INPUT
%     t0: [nTraj] original time vector.
%     yd: [nTraj] positions
%     
%     param.trajFraction = 0.1; percentage of the trajectory to be modified
%                    0.1 means that the first 10% of the time vector will
%                    be modified
%     param.dtTry = 0.001:0.01:0.5; the possible values of DeltaT to be
%                    added to the original dt. The value represents only
%                    the first dt added on the top of the original dt. It
%                    then decays to zero, such that the original dt of the
%                    trajectory is recovered after, say 10%
%
%     param.minInitVel = 0.10; the goal initial velocity.
%
% param.trajFraction = 0.1;
% param.dtTry = 0.001:0.01:0.5;
% param.minInitVel = 0.10; 
%

    dbg=1;
    goalSatisfied=0;
    nTraj = numel(t0);    
    dtTry = param.dtTry;
    
    idxf = round(nTraj*param.trajFraction);
    % deceleration profile as variable dT
    t_ = 1:idxf;
    t_ = t_/idxf;
    dtNewPartialNormalized = exp(-5*t_)';    
    dtNewPartialNormalized = dtNewPartialNormalized./dtNewPartialNormalized(1);
    
    
    dt0   = diff(t0);
    dt0   = [dt0 ; dt0(end)];
        
    fig(1).h = figure; hold on; title('Additional delta T');  grid on;    
    fig(1).ploth =[];
    if dbg            
        figure(fig(1).h);            
        fig(1).ploth = [fig(1).ploth     plot(t_, dtNewPartialNormalized)];
    end    

    fig(2).h = figure;hold on;grid on; title('New delta T');    
    fig(2).ploth =[];
    
    fig(3).h = figure; 
    subplot(2,1,1); hold on; grid on; ylabel 'position';
    subplot(2,1,2); hold on; grid on; ylabel 'velocity';
    
    maxIter = numel(dtTry); 
    
    
    switch type
        case 'start'
            idxRef = 1;
        case 'end'
            idxRef = nTraj-1;
    end
    
    for k=1:maxIter
        for j=1:numel(fig)
            try
                delete(fig(j).ploth);
            end
        end
        dtNewPartial = dtNewPartialNormalized*dtTry(k);

        if strcmp(type, 'end')
            dtNewPartial = dtNewPartial(end:-1:1);
        end
        
        if dbg
            figure(fig(1).h);
            fig(1).ploth = [fig(1).ploth      plot(t_, dtNewPartial, 'r')];    
        end

        if strcmp(type, 'start')
            dtNew = [dtNewPartial+dt0(1:numel(dtNewPartial)) ; dt0(numel(dtNewPartial)+1:end)  ];
        else % optimize the end
            dtNew = [dt0(1:nTraj-numel(dtNewPartial)) % original part
                     dt0(nTraj-numel(dtNewPartial)+1:end)+dtNewPartial];
        end

        if dbg
            figure(fig(2).h);
            fig(2).ploth = [fig(2).ploth  plot(dt0, 'bo-')];
            fig(2).ploth = [fig(2).ploth  plot(dtNew, 'r.-')];
            legend({'original dt', 'new dt'});
        end

        tNewFull = cumsum(dtNew);
        tNewFull = [0 ; tNewFull(1:end-1)];

        vel=diff(yd)./diff(t0);
        velNew = diff(yd)./diff(tNewFull);

        if dbg    
            figure(fig(3).h);
            subplot(2,1,1);
            fig(3).ploth = [fig(3).ploth  plot(t0, yd, 'b')];
            fig(3).ploth = [fig(3).ploth  plot(tNewFull, yd, 'r')];
            subplot(2,1,2);
            fig(3).ploth = [fig(3).ploth  plot(t0(1:end-1), vel, 'b')];
            fig(3).ploth = [fig(3).ploth  plot(tNewFull(1:end-1), velNew, 'r')];    
        end
        drawnow;

        fprintf('Vel. orig/curr %g   %g.\n', vel(idxRef), velNew(idxRef));
        if abs(param.minInitVel) > abs(velNew(idxRef))
            fprintf('\n\n***Goal satisfied.\n');
            goalSatisfied=1;
            break
        end

    end
            
            
if 0
            
    switch type
        case 'start'

            for k=1:maxIter

                for j=1:numel(fig)
                    try
                        delete(fig(j).ploth);
                    end
                end
                dtNewPartial = dtNewPartialNormalized*dtTry(k);

                if dbg
                    figure(fig(1).h);
                    fig(1).ploth = [fig(1).ploth      plot(t_, dtNewPartial, 'r')];    
                end

                dtNew = [dtNewPartial+dt0(1:numel(dtNewPartial)) ; dt0(numel(dtNewPartial)+1:end)  ];

                if dbg
                    figure(fig(2).h);
                    fig(2).ploth = [fig(2).ploth  plot(dt0, 'bo-')];
                    fig(2).ploth = [fig(2).ploth  plot(dtNew, 'r.-')];
                    legend({'original dt', 'new dt'});
                end

                tNewFull = cumsum(dtNew);
                tNewFull = [0 ; tNewFull(1:end-1)];

                vel=diff(yd)./diff(t0);
                velNew = diff(yd)./diff(tNewFull);

                if dbg    
                    figure(fig(3).h);
                    subplot(2,1,1);
                    fig(3).ploth = [fig(3).ploth  plot(t0, yd, 'b')];
                    fig(3).ploth = [fig(3).ploth  plot(tNewFull, yd, 'r')];
                    subplot(2,1,2);
                    fig(3).ploth = [fig(3).ploth  plot(t0(1:end-1), vel, 'b')];
                    fig(3).ploth = [fig(3).ploth  plot(tNewFull(1:end-1), velNew, 'r')];    
                end
                drawnow;

                fprintf('Vel. orig/curr %g   %g.\n', vel(1), velNew(1));

                if abs(param.minInitVel) > abs(velNew(1))
                    fprintf('\n\n***Goal satisfied.\n');
                    goalSatisfied=1;
                    break
                end

                %pause(0.1);
            end
  
        case 'end'
            

            for k=1:maxIter

                for j=1:numel(fig)
                    try
                        delete(fig(j).ploth);
                    end
                end
                dtNewPartial = dtNewPartialNormalized*dtTry(k);
                
                % reverse
                dtNewPartial = dtNewPartial(end:-1:1);

                if dbg
                    figure(fig(1).h);
                    fig(1).ploth = [fig(1).ploth      plot(t_, dtNewPartial, 'r')];    
                end
                
                
                % reverse
                dtNew = [dt0(1:nTraj-numel(dtNewPartial)) % original part
                         dt0(nTraj-numel(dtNewPartial)+1:end)+dtNewPartial];
            
                if dbg
                    figure(fig(2).h);
                    fig(2).ploth = [fig(2).ploth  plot(dt0, 'bo-')];
                    fig(2).ploth = [fig(2).ploth  plot(dtNew, 'r.-')];
                    legend({'original dt', 'new dt'});
                end

                tNewFull = cumsum(dtNew);
                tNewFull = [0 ; tNewFull(1:end-1)];

                vel=diff(yd)./diff(t0);
                velNew = diff(yd)./diff(tNewFull);

                if dbg    
                    figure(fig(3).h);
                    subplot(2,1,1);
                    fig(3).ploth = [fig(3).ploth  plot(t0, yd, 'b')];
                    fig(3).ploth = [fig(3).ploth  plot(tNewFull, yd, 'r')];
                    subplot(2,1,2);
                    fig(3).ploth = [fig(3).ploth  plot(t0(1:end-1), vel, 'b')];
                    fig(3).ploth = [fig(3).ploth  plot(tNewFull(1:end-1), velNew, 'r')];    
                end
                drawnow;

                % 
                fprintf('Vel. orig/curr %g   %g.\n', vel(1), velNew(1));

                % reverse
                if abs(param.minInitVel) > abs(velNew(end))
                    fprintf('\n\n***Goal satisfied.\n');
                    goalSatisfied=1;
                    break
                end

                %pause(0.1);
            end
            
            
            
            
    end
            
end

    if goalSatisfied==0
        fprintf('\n\n***Max iterations exceeded.\n');
    end
    
end



% 
% 
% 
%     dbg=1;
% 
%     nTraj = numel(t0);
%     
%     
%     
%     dtExtra = param.dtMax;
%     
%     
%     
%     idxf = round(nTraj*param.trajFraction);
% 
%     % deceleration profile as variable dT
%     t_ = 1:idxf;
%     t_ = t_/idxf;
%     dtNewPartial = exp(-5*t_);
%     
%     if dbg
%         fig(1).h = figure; hold on;
%         title('Additional delta T');
%         grid on;
%         plot(t_, dtNewPartial);
%     end
%     
%     dtNewPartial = dtNewPartial*dtExtra;
%     
%     if dbg
%         figure(fig(1).h);
%         plot(t_, dtNewPartial, 'r');    
%     end
%     
%     dt0   = diff(t0);
%     dt0   = [dt0  dt0(end)];
%     dtNew = [dtNewPartial+dt0(1:numel(dtNewPartial))  dt0(numel(dtNewPartial)+1:end)  ];
%     
%     if dbg
%         figure;hold on;grid on;
%         title('New delta T');
%         plot(dt0, 'bo-');
%         plot(dtNew, 'r.-');
%         legend({'original dt', 'new dt'});
%     end
%     
%     tNewFull = cumsum(dtNew);
%     tNewFull = [0 tNewFull(1:end-1)];
%     
%     vel=diff(yd)./diff(t0);
%     velNew = diff(yd)./diff(tNewFull);
%     if dbg    
%         figure; subplot(2,1,1); hold on;
%         plot(t0, yd);
%         plot(tNewFull, yd, 'r');
%         subplot(2,1,2); hold on;
%         plot(t0(1:end-1), vel);
%         plot(tNewFull(1:end-1), velNew, 'r');    
%     end
%     
%     fprintf('Initial vel orig %g\n', vel(1));
%     fprintf('Initial vel opti %g\n', velNew(1));
% 
% 



