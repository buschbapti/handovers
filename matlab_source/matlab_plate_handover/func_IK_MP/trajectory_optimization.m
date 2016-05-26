function [solutionRobotFinal, sysNew, key_name] = trajectory_optimization(robot, demo, param, init,...
                          hworkspc, key_name)

% check if key_name exist, if so just reload the solution
file_names    = get_folder_names('./');
if  sum(strcmp(file_names, key_name))~= 0
    
    disp('Reloading solution.');
    disp('Animation playing in VREP.');

    [solutionRobotFinal, sysNew, param] = recover_final_solution(key_name);

    clear param;
    param.hfig = figurewe(['reloaded_' key_name]);
    param.axis_length = 0.075;
    param.nMaxPlots = 5;
    param.shadowHeight     = -1;
    param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
    homogTransfPlot(robot.initialCartesianTrajectory.T, param);


    plot3( demo.vpAppr.T(1,4), demo.vpAppr.T(2,4), demo.vpAppr.T(3,4), style('g', 'o', 1, '-', 10) );
    homogTransfPlot(demo.vpAppr.T, param);

    param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'k', 'Marker', 'none');
    param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .75 .75]);
    param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.75 1 .75]);
    param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.75 .75 1]);
    homogTransfPlot(solutionRobotFinal.T, param);

    param.axis_length = 0.10;
    param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 .5 .5]);
    param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [.5 1 .5]);
    param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [.5 .5 1]);

    homogTransfPlot(robot.TrestPosture, param);
    homogTransfPlot(robot.initialCartesianTrajectory.T(:,:,end), param);
    view([1 -1  0.5])


else

    ProMP = createProMP(init, []); % create on the shifted trajectory already

    [param, robot]  = create_main_figures(param, robot, init);
    sys             = sys_structure(ProMP, param, []  ); % create structure for ref frame parameters

    % Define the via points
    % ========================================================
    %figHandles = [ hworkspc, param.plot_handles.single_IK_iteration, param.hfig.noisyRollOuts ];           
    figHandles  = [ hworkspc, param.hfig.single_IK_iteration, param.hfig.noisyRollOuts ];
    [init.robot, param.mp] = set_via_point_and_trajectory_poses(0, ...
                                    robot, init.robot, demo.vpAppr, figHandles);

    [init.robotRes, sys{1}.resampleIdx]  = resample_for_IK(init.robot, param);
    param.reps.Weight_IK_MP = [1  1]; % control the cost of each part

    %try close(hworkspc); end    
    
    % prepare for first iteration
    sysNew        = sys;
    initGuessTNew = init.robotRes;
    paramNew      = param; 
    
    disp('Computing solution')
    
    ib = 1;
    [sysNew, initGuessTNew, finishFlag] = optimization_loop(sysNew, robot, initGuessTNew, paramNew );
    dataBatch{ib} = DataSaver([key_name ],  sysNew, initGuessTNew, init.robot.T, paramNew ); 
    DataSaver.saveCurrent(dataBatch{ib}); 
    sspng(['./' dataBatch{ib}.fileName], dataBatch{ib}.fileName );       
    
    
    [solutionRobotFinal, sysNew, param] = recover_final_solution(dataBatch{ib}.fileName);
    
    key_name = dataBatch{ib}.fileName;
        
end













