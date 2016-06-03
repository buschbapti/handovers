function [T, screw] = load_screw_data()

    if 0  % checking human data
        % load the shelf trajectories
        load('../screw_1human_dta/grasp_screw_set1.mat');

        param.hfig = cartesian_plot('shelf_human_traj_raw');
        param.axis_length = 0.05;
        param.nMaxPlots = 20;
        param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
        param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .5 .5]);
        param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 1 .5]);
        param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 1]);
        param.shadowHeight     = -0.6;
        T = T2;
        
        for k=1:10
            homogTransfPlot(  T{k}, param );
            text(T{k}(1,4,end), T{k}(2,4,end), T{k}(3,4,end), num2str(k));
        end

        % visual inspection
        figurew('timedata'); 
        subplot(3,1,1); hold on; grid on;
        subplot(3,1,2); hold on; grid on;
        subplot(3,1,3); hold on; grid on;
        for k=1:10
            for j=1:3
                subplot(3,1,j);
                plot(squeeze(T{k}(j,4,:)), 'bo-');
                text(numel(squeeze(T{k}(j,4,:)))+5, squeeze(T{k}(j,4,end)),  num2str(k));
            end
        end
    
        % cut trajectories
        param.hfig = cartesian_plot('shelf_human_traj');
        for k=1:10
            T{k} = T{k}(:,:,21:100);
            homogTransfPlot(  T{k}, param );
            text(T{k}(1,4,end), T{k}(2,4,end), T{k}(3,4,end), num2str(k));            
        end        
        save('T_human_demo.mat', 'T');        
    else
        load('T_human_demo.mat');        
    end
    
    if 1 % load location of shelf in darias workspace
        load('../../generic_robot_commands/screw_desired_joint_data/screw_coordinates_in_vrep.mat');
    end
    

end


