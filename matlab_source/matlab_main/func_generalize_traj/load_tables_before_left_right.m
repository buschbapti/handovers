function lookupTraj = load_tables(folderName)

    folder = ['./' folderName];
        
    if strcmp(folderName, 'lookupTraj1')
        %load([folder '/table1_20151111_113045.mat']);
        load([folder '/table1_20151130_121146.mat']);        
        lookupTraj{1} = solTraj1;
        
        %load([folder '/del1_20151112_104318.mat']);
        %lookupTraj{end+1} = solTraj1;       
    end

    if strcmp(folderName, 'lookupTraj2')
        
        dbg = 0;
        if dbg
            param.hfig = figurewe('checkTraj');
            param.axis_length = 0.05;
            param.nMaxPlots = 10;
            param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
            param.axesPlotStyle{1} = struct('LineWidth', 3, 'Color', [1 .5 .5]);
            param.axesPlotStyle{2} = struct('LineWidth', 3, 'Color', [.5 1 .5]);
            param.axesPlotStyle{3} = struct('LineWidth', 3, 'Color', [.5 .5 1]);
            param.shadowHeight     = -1;
            view([1 -1 2]);
            
            hall = figurewe('checkSimple');
            view([1 -1 2]);
        end
        fileNames = get_file_names(['./' folderName]);
        
        k=1;
        for j=1:numel(fileNames)
            if findstr(fileNames{j}, 'reba')
                load([folder '/' fileNames{j}]);
                lookupTraj{k} = solTraj2;                
                if dbg
                    [~, hCurves] = homogTransfPlot(lookupTraj{k}.sol.T, param );
                    figure(hall);
                    plot3(lookupTraj{k}.sol.p(1,:), lookupTraj{k}.sol.p(2,:), lookupTraj{k}.sol.p(3,:));
                    text(lookupTraj{k}.sol.p(1,end), lookupTraj{k}.sol.p(2,end), lookupTraj{k}.sol.p(3,end), num2str(k));
                    pause
                    delete(hCurves);
                end
                k=k+1;
            end
        end
        
        %REBA: 0.629141532883	-0.177461599313	-0.109639630416	0.737865791191	-0.186749523442	-0.596509116497	0.2546675551	
        %load([folder '/table2reba_20151130_161153.mat']);  % good
        
        
        
        
    end


end