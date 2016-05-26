function [param, robot] = create_main_figures(param, robot, init)
% function param = create_main_figures(param, computerName)

%% Simpli select which types of figure you want to plot

    figureControl.mainFig = 0;
    figureControl.noisyRollouts = 0;
    figureControl.ikineError_vs_time= 0;
    
    %% You should not worry about this code

    
    param.hfig.costFrame = figurew('cost'); 
    subplot(3,1,1); hold on; grid on; xlim([0 param.reps.nUpdates+1]); ylabel 'invkin';
    subplot(3,1,2); hold on; grid on; xlim([0 param.reps.nUpdates+1]); ylabel 'traj';
    subplot(3,1,3); hold on; grid on; xlim([0 param.reps.nUpdates+1]); ylabel 'viaPoint error (cm)';
    plot([0 param.reps.nUpdates], [0 0 ], 'k');
    ylim([0 40]);
    
    
    % Parameters for plotting
    if figureControl.noisyRollouts
        param.hfig.noisyRollOuts = figurewe('noisy_rollouts'); axis 'equal'; 
        view([0   0   1 ]); box off; grid on; 
        xlabel 'x'; ylabel 'y'; zlabel 'z';
    else
        param.hfig.noisyRollOuts = [];
    end
    param.hfig.frame_exploration = param.hfig.noisyRollOuts;    
%     param.plot_handles.main =param.plot.frame0.hfig;
%     param.plot.frame_currentSol.hfig = param.plot.frame0.hfig;
%     param.plot.frame_leaveTrace.hfig = param.plot.frame0.hfig;

    
    if figureControl.ikineError_vs_time
        param.hfig.ikine_error = figurew('ikine_error'); grid off;
    else
        param.hfig.ikine_error = [];
    end

    robot.opt.plotShadowzheight = -0.475;
    shadowHeight = robot.opt.plotShadowzheight;
    restPosture  = robot.TrestPosture;
    if figureControl.mainFig
        param.hfig.single_IK_iteration = prepare_fig_iteration(init.robot.p, ...
                                                       shadowHeight, restPosture);          
    else
        param.hfig.single_IK_iteration = [];
    end
    

    
    
end
    

    




