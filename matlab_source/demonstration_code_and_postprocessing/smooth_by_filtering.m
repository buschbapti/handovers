function [y] = smooth_by_filtering(q, param)

    forder = param.filterOrder;
    freq   = param.filterFreq;
    
    h(1) = figurew('q');
    set_fig_position([0.272 0.0407 0.298 0.931]);
    for j=1:numel( q(1,:) )
        subplot(numel( q(1,:) ),1,j); grid on; hold on;
        plot(q(:,j), 'b.-');
        [b,a] = butter( forder, freq,  'low'); % IIR filter design
        y(:,j) = filtfilt(b,a,q(:,j));    
        plot(y(:,j), 'r.-');
    end
    
    if 0
        h(2) = figurew('qdot');
        set_fig_position([0.537 0.0435 0.298 0.932]);
        for j=1:numel( q(1,:) )
            subplot(numel( q(1,:) ),1,j); grid on; hold on;
            plot(diff(q(:,j)), 'b.-');
            plot(diff(y(:,j)), 'r.-');
        end
    end
    
end