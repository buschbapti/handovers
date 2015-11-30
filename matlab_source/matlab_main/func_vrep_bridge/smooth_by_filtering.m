function [y] = smooth_by_filtering(q, param, plotFlag)

    if ~exist('plotFlag', 'var')
        plotFlag=0;
    end

    forder = param.filterOrder;
    freq   = param.filterFreq;
    
    if plotFlag
        h(1) = figurew('q');
        set_fig_position([0.272 0.0407 0.298 0.931]);
    end
    
    
    nDof = numel(q(1,:));
    for j=1:nDof
        if plotFlag
            subplot(nDof,1,j); grid on; hold on;
            plot(q(:,j), 'b.-');
        end
        [b,a] = butter( forder, freq,  'low'); % IIR filter design
        y(:,j) = filtfilt(b,a,q(:,j));    
        if plotFlag
            plot(y(:,j), 'r.-');
        end
    end
    
    if plotFlag
        h(2) = figurew('qdot');
        set_fig_position([0.537 0.0435 0.298 0.932]);
        for j=1:nDof
            subplot(nDof,1,j); grid on; hold on;
            plot(diff(q(:,j)), 'b.-');
            plot(diff(y(:,j)), 'r.-');
        end
    end
    
end