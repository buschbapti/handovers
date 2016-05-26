function [ProMP] = createProMP(init, hfig)

    plot_flag=0;

    init_ias{1,:} = bsxfun(@minus,init.robot.p,[0.1; 0.1; 0.1])';
    init_ias{2,:} = bsxfun(@plus,init.robot.p,[0.1; 0.1; 0.1])';

    % create ProMP here
    ProMP = regression_pmp(init_ias, 15, 'GaussianFeatures');

    % plot the initial distribution    
    for j=1:ProMP.nJoints
        
        ir = ProMP.w.index{j};
        Gn = ProMP.basis.Gn;
        Gndot = ProMP.basis.Gndot;
        Sigma_model = ProMP.w.cov_full(ir, ir);
        SigmaX      = Gn*Sigma_model*Gn';
        meanX       = Gn*ProMP.w.mean_full(ir);
        meanXdot    = Gndot*ProMP.w.mean_full(ir);
        
        if plot_flag
            figurew(['joint', num2str(j)]);
            hShade = shadedErrorBar([1:ProMP.nTraj], meanX , 2*sqrt( diag(SigmaX) ), {'g', 'LineWidth', 2'}, 1);
            plot(init_ias{1}(:,j), 'b.-');
            plot(init_ias{2}(:,j), 'r.-');
            %delete(hShade.mainLine); delete(hShade.edge);
        end
        givenTraj.xyz(:,j)    = meanX;
        givenTraj.xyzdot(:,j) = meanXdot;
    end
    
    ProMP.encodedMeanTraj = givenTraj.xyz;
    
    if plot_flag
        figurewe('3Dcheck');
        plot3(init_ias{1}(:,1), init_ias{1}(:,2), init_ias{1}(:,3), 'b-');
        plot3(init_ias{2}(:,1), init_ias{2}(:,2), init_ias{2}(:,3), 'r-');
        plot3(givenTraj.xyz(:,1),givenTraj.xyz(:,2),givenTraj.xyz(:,3), 'k.-');
    end
    
    if ~isempty(hfig)
        figure(hfig)
        hdel = plot3(ProMP.encodedMeanTraj(:,1), ProMP.encodedMeanTraj(:,2), ProMP.encodedMeanTraj(:,3), ...
                     sty('g', [], 10));        
    end
    
    
end