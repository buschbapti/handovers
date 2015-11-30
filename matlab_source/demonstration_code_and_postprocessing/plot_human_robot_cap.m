

function [] = plot_human_robot_cap(dataMocap, scale, plotLabel)

    
    param.hfig = gcf;
    param.axis_length = scale;
    param.nMaxPlots =  25;
    param.pathPlotStyle =  styw([0.6 0.6 0.6], 'none', 1.25, '-', 5);
    
    if isfield(dataMocap, 'robotT')
        if ~isempty(dataMocap.robotT)
            agent.T = dataMocap.robotT;
            agent.p = squeeze(dataMocap.robotT(1:3,4,:))';
            plott(agent, param, plotLabel);
        end
    end
    if isfield(dataMocap, 'capT')
        if ~isempty(dataMocap.capT)
            agent.T = dataMocap.capT;
            agent.p = squeeze(dataMocap.capT(1:3,4,:))';
            plott(agent, param, plotLabel);
        end
    end
    if isfield(dataMocap, 'humanT')
        if ~isempty(dataMocap.humanT)
            agent.T = dataMocap.humanT;
            agent.p = squeeze(dataMocap.humanT(1:3,4,:))';
            plott(agent, param, plotLabel);
        end
    end
    
end


function [] = plott(agent, param, plotLabel)

    tableHeight = -1.22;
    homogTransfPlot(agent.T, param);
    plot3(agent.p(:,1), agent.p(:,2),tableHeight*ones(size(agent.p(:,1))), sty_nl([0.8 0.8 0.8],[],2,'-',[])) ;    
    if plotLabel
        text(agent.T(1,4,1)+0.1, agent.T(2,4,1), agent.T(3,4,1), 'robot' );
    end

end





