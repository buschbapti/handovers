function lookupTrajL = load_tables_L()

    folderName = 'lookupTraj2_LR';    
       
    lookupTrajL = get_traj(folderName, 'left', 0);
  %  lookupTraj.left  = lookupTrajL;

  %  lookupTrajR = get_traj(folderName, 'right', 0);
  %  lookupTraj.right  = lookupTrajR;

end

function [lookupTrajSelected] = get_traj(folderName, side, dbg)

    folder = ['./' folderName];
    
    if dbg
        param.hfig = figurewe(['checkTraj_' side]);
        set_fig_position([0.246 0.215 0.382 0.611]);
        param.axis_length = 0.05;
        param.nMaxPlots = 10;
        param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
        param.axesPlotStyle{1} = struct('LineWidth', 3, 'Color', [1 .5 .5]);
        param.axesPlotStyle{2} = struct('LineWidth', 3, 'Color', [.5 1 .5]);
        param.axesPlotStyle{3} = struct('LineWidth', 3, 'Color', [.5 .5 1]);
        param.shadowHeight     = -1;
        view([1 -1 2]);

        hall = figurewe(['checkSimple_' side ] );
        set_fig_position([0.588 0.0964 0.375 0.572]);
        view([1 -1 2]);
    end
    fileNames = get_file_names(['./' folderName]);

    k=1;
    for j=1:numel(fileNames)
        if findstr(fileNames{j}, ['reba_' side])
            load([folder '/' fileNames{j}]);
            lookupTraj{k} = solTraj2;                
            if dbg
                [~, hCurves] = homogTransfPlot(lookupTraj{k}.sol.T, param );
                title(['Number ', num2str(k)]);
                figure(hall);
                plot3(lookupTraj{k}.sol.p(1,:), lookupTraj{k}.sol.p(2,:), lookupTraj{k}.sol.p(3,:));
                text(lookupTraj{k}.sol.p(1,end), lookupTraj{k}.sol.p(2,end), lookupTraj{k}.sol.p(3,end), num2str(k));
                pause
                delete(hCurves);
            end
            k=k+1;
        end
    end
    
    % Select the trajectories that make sense
    if strcmp(side, 'left') % check 17
        keepSolutions = [1 2  4 5 6 7 8    9 10 11  20 21  22 ];
    end
        
    if strcmp(side, 'right')
        keepSolutions =  [2 3 16 17 18 19];
    end
    
    ctr=1;
    for k=1:numel(keepSolutions)
        lookupTrajSelected{ctr} = lookupTraj{keepSolutions(k)};
        ctr = ctr+1;
    end
        
    
    
    
end




















