function [param, robot] = create_main_figures(param, robot, init, computerName)
% function param = create_main_figures(param, computerName)

%% Simpli select which types of figure you want to plot

    figureControl.mainFig = 0;
    figureControl.noisyRollouts = 0;
    figureControl.ikineError_vs_time= 0;
    
    %% You should not worry about this code

    
    param.plot_handles.costFrame = figurew('cost'); 
    subplot(3,1,1); hold on; grid on; xlim([0 param.reps.nUpdates+1]); ylabel 'invkin';
    subplot(3,1,2); hold on; grid on; xlim([0 param.reps.nUpdates+1]); ylabel 'traj';
    subplot(3,1,3); hold on; grid on; xlim([0 param.reps.nUpdates+1]); ylabel 'viaPoint error (cm)';
    plot([0 param.reps.nUpdates], [0 0 ], 'k');
    ylim([0 40]);
    
    
    % Parameters for plotting
    if figureControl.noisyRollouts
        param.plot.frame0.hfig = figurewe('noisy_rollouts'); axis 'equal'; 
        view([0   0   1 ]); box off; grid on; 
        xlabel 'x'; ylabel 'y'; zlabel 'z';
    else
        param.plot.frame0.hfig = [];
    end
    param.plot_handles.main =param.plot.frame0.hfig;
    param.plot.frame_currentSol.hfig = param.plot.frame0.hfig;
    param.plot.frame_leaveTrace.hfig = param.plot.frame0.hfig;
    param.plot.frame_exploration.hfig = param.plot.frame0.hfig;
    
    if figureControl.ikineError_vs_time
        param.plot_handles.ikine_error = figurew('ikine_error'); grid off;
    else
        param.plot_handles.ikine_error = [];
    end

    robot.opt.plotShadowzheight = -0.475;
    shadowHeight = robot.opt.plotShadowzheight;
    restPosture  = robot.TrestPosture;
    if figureControl.mainFig
        param.plot_handles.single_IK_iteration = prepare_fig_iteration(init.robot.p, ...
                                                       shadowHeight, restPosture);          
    else
        param.plot_handles.single_IK_iteration = [];
    end
    

    switch  computerName.name
        
        case 'v5i7-Aspire-V5-571G'
            
            
        case 'mito-ub-vaio'
            
            if strcmp(computerName.layout, 'vertical')
                
                figure(param.plot.frame0.hfig);
                set_fig_position([0.0396 0.775 0.244 0.225]);

                figure(param.plot_handles.ikine_error);
                set_fig_position([0.398 0.441 0.39 0.207]);

                figure(param.plot_handles.costFrame);
                set_fig_position([0.717 0.468 0.253 0.442]);

                %figure(param.plot_handles.currentIKMPsol);
                set_fig_position([0.298 0.0823 0.295 0.312]);

                figure(param.plot_handles.single_IK_iteration);
                set_fig_position([0.292 0.668 0.373 0.321]);
            end
                
            if strcmp(computerName.layout, 'horizontal')
                
                figure(param.plot.frame0.hfig);
                set_fig_position([0.518 0.519 0.164 0.445]);

                figure(param.plot_handles.ikine_error);
                set_fig_position([0.706 0.0176 0.155 0.435]);

                figure(param.plot_handles.costFrame);
                set_fig_position([0.862 0.0213 0.107 0.731]);

                %figure(param.plot_handles.currentIKMPsol);
                set_fig_position([0.553 0.037 0.131 0.506]);

                figure(param.plot_handles.single_IK_iteration);
                set_fig_position([0.683 0.519 0.168 0.447]);
            end
            
        case 'gjmasus'

            figure(param.plot_handles.costFrame);
            set_fig_position([0.349 0.0935 0.3 0.89]);

                  
            
        otherwise          
            
    end
    
    
end
    

    




