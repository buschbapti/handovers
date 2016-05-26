function plotS = userParamPlot(nUpdates)

    plotS.frame0.axis_length  = 0.025;
    plotS.frame0.nMaxPlots    = 15;

    gray = [0.7 0.7 0.7;];
    plotS.frame_currentSol = plotS.frame0;
    plotS.frame_currentSol.pathPlotStyle    = struct('LineWidth', 2, 'Color', [1 0.5 0], 'Marker', 'o', 'MarkerFaceColor', 'auto');
    plotS.frame_currentSol.axesPlotStyle{1} = struct('Marker', 'o','LineWidth', 0.1, 'Color', 'r', 'LineStyle', '--');
    plotS.frame_currentSol.axesPlotStyle{2} = struct('Marker', 'o', 'LineWidth', 0.1, 'Color', 'g', 'LineStyle', '--');
    plotS.frame_currentSol.axesPlotStyle{3} = struct('Marker', 'o','LineWidth', 0.1, 'Color', 'b', 'LineStyle', '--');
    plotS.frame_currentSol.axis_length = plotS.frame0.axis_length + 0.01;
    plotS.frame_currentSol.plotflag = 0; % deactivate the plot

    plotS.frame_leaveTrace                  = plotS.frame_currentSol;
    plotS.frame_leaveTrace.pathPlotStyle    = struct('LineWidth', 0.1, 'Color', gray, 'Marker', '.');
    plotS.frame_leaveTrace.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', 'r', 'LineStyle', '-');
    plotS.frame_leaveTrace.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', gray, 'LineStyle', '-');
    plotS.frame_leaveTrace.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', gray, 'LineStyle', '-');

    plotS.frame_exploration                  = plotS.frame_leaveTrace;
    plotS.frame_exploration.pathPlotStyle    = struct('LineWidth', 1,   'Color',   'r', 'Marker', 'none');
    plotS.frame_exploration.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', 'r', 'LineStyle', '-');
    plotS.frame_exploration.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', 'r', 'LineStyle', '-');
    plotS.frame_exploration.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', 'r', 'LineStyle', '-');
    plotS.frame_exploration.handleCurves =[];

    plotS.animateEveryIteration = 0;

    plotS.colorGrad = gradientOfColor(nUpdates, 'gray');
    

end