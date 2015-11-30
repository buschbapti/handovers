function T3 = shift_marker_on_hand_center(T, param)
% This function is specific to calibrate the marker on the back of the hand
% to be inside the palm of the hand and alighned with the grasping of the
% fingers.
%
% Note that xyz offsets are specified in the reference of the marker
% 
%
    % 1. shift marker to be inside hand
    % ===========================================
    xyzShift1 = [param.xyzShift(1) 0 0]';
    T1 = move_XYZ_on_intrinsic_frame(T, xyzShift1);

    % Now rotate the final frame to align the Z axis with the fingers of
    % the hand
    [Ty, Ry] = my_troty( param.angleAroundY );
    T2 = T1*Ty;

    % Finally move in the Z direction
    xyzShift2 = [0 0 param.xyzShift(3)]';
    T3 = move_XYZ_on_intrinsic_frame(T2, xyzShift2);
    
    dbg = 0;
    if dbg
        
        prmplot1.hfig = figurewe('rotated');     view([1 -1 1]);    
        prmplot1.axis_length = 0.05;
        prmplot1.nMaxPlots = 10;
        prmplot1.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'b', 'Marker', 'o');
        prmplot1.axesPlotStyle{1} = struct('LineWidth', 4, 'Color', lightRGB(1));
        prmplot1.axesPlotStyle{2} = struct('LineWidth', 4, 'Color', lightRGB(2));
        prmplot1.axesPlotStyle{3} = struct('LineWidth', 4, 'Color', lightRGB(3)); 
        homogTransfPlot(T, prmplot1);
        
        prmplot2 = prmplot1;
        prmplot2.axis_length = 0.025;
        prmplot2.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 0 0]);
        prmplot2.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [0 1 0]);
        prmplot2.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [0 0 1]);         
        homogTransfPlot(T1, prmplot2);
        
        prmplot2 = prmplot1;
        prmplot2.axis_length = 0.015;
        prmplot2.axesPlotStyle{1} = struct('LineWidth', 8, 'Color', [1 0 0]);
        prmplot2.axesPlotStyle{2} = struct('LineWidth', 8, 'Color', [0 1 0]);
        prmplot2.axesPlotStyle{3} = struct('LineWidth', 8, 'Color', [0 0 1]);         
        homogTransfPlot(T2, prmplot2);
        homogTransfPlot(T3, prmplot2);
    end
    

    

end















