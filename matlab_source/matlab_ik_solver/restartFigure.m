function h = restartFigure(xyzOriginal, T, obst, vp, robot)

    gtxyzTraj = squeeze(T(1:3,4,:));
    h = createFig([], gtxyzTraj, [], vp);
   % h = createFig([], squeeze(T(1:3,4,:)), [], vp);
    
    figure(h.fig);
    try
        homogTransfPlot(vp.mid.T, struct('hfig', h.fig, 'axis_length', 0.15));    
    end
    
    %homogTransfPlot(T, struct('hfig',h.fig));
    
    
end