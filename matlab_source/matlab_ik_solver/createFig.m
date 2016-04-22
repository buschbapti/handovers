function h = createFig(gtxyzTraj, xyzOriginal, obst, vp )

    h.figcost    = figurew('cost_DMP');
    set_fig_position([0.0781 0.682 0.202 0.28]);
    
    h.figcostIK    = figurew('cost_IK'); set_fig_position([0.264 0.025 0.293 0.776]);
    subplot(2,1,1); grid on; hold on;
    subplot(2,1,2); grid on; hold on;

    h.fig = figurewe('Cartesian'); 
    set_fig_position([0.558 0.0722 0.382 0.895]);
    plot3(xyzOriginal(1,:), xyzOriginal(2,:), xyzOriginal(3,:), sty(lightRGB(3), [], 0.015, '-'));
    if ~isempty(gtxyzTraj)
        plot3(gtxyzTraj(1,:), gtxyzTraj(2,:), gtxyzTraj(3,:), sty([0.9 0.9 0.9], 'o', 0.1, 'none', 3 ) );
    end
    % table
    plot_workspace(h.fig);
%     xlim([-0 0.8]); 
%     ylim([-1.0 -0.2])
    
    
    % plot start and end
    param.hfig = h.fig;
    param.axis_length = 0.05;
    param.nMaxPlots = 20;
    param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'o', 'MarkerSize', 10);
    param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 .0 .0]);
    param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [.0 1 .0]);
    param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [.0 .0 1]);
    param.shadowHeight     = -0.6;
    homogTransfPlot(vp.init, param);
    
    param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'x', 'MarkerSize', 10);
    homogTransfPlot(vp.finl, param);
    
end