function [init2, mp] = set_via_point_and_trajectory_poses(minCost, robot, init, viapoint, hfig)
%
% INPUT
%   viaPointIdx: [nVP x 1] where each value is the index of the via point
%
%   viaPointVal: [nVP x 3]
%                 nVP : number of via points as xyz coordinates
%
%    init:  the initial robot trajectory
%
%
% OUTPUT
%    init2: the initial robot trajectory where the orientation at each state is
%           adjusted to the final desired one, that is, the initial orientation
%           coincides with the current robot hand pose. The orientation of
%           the via point and handover are also already forced to be at the
%           desired ones. 
%           By doing this the trajectory optimization does not have deal in
%           optimizing orientations as they are already defined.
%         

    mp.minCost = minCost; % the cost that will stop the optimization
        
    q    = NaN.*ones(numel(init.p(1,:)),3);
    qdot = NaN.*ones(numel(init.p(1,:)),3);
    
    % adding the initial rest posture as via point but only if does not
    % exist
    if sum((viapoint.idx-1)==0)==0
        viapoint.idx(end+1)   = 1;
        viapoint.T(:,:,end+1) = robot.TrestPosture;
    end

    % adding the final hand over as via point but only add if it does not
    % exist already
    if sum(viapoint.idx-numel(init.p(1,:))==0)==0
        viapoint.idx(end+1)   = numel(init.p(1,:));    
        viapoint.T(:,:,end+1) = init.T(:,:,end);
    end
    
    % sorting the order of the via points 
    [a, b] = sort(viapoint.idx);    
    vp.idx = a;
    vp.T   = viapoint.T(:,:,b);    
    viapoint = vp;
    
    nvp = numel(viapoint.idx); 
    
    for iv = 1:nvp
        idx = viapoint.idx(iv);
        q(idx,:) = viapoint.T(1:3,4,iv)'; 
        
        if ~isempty(hfig)
            for ih=1:numel(hfig)
                figure(hfig(ih));
                plot3(viapoint.T(1,4,iv), viapoint.T(2,4,iv), viapoint.T(3,4,iv), ...
                          sty_nl('k', 'x', 2, [], 10));
                plot3(viapoint.T(1,4,iv), viapoint.T(2,4,iv), robot.opt.floorHeight, ...
                          sty_nl([0.6 0.6 0.6], 'x', 2, [], 10));
                if iv==1
                    string_ = 'rest';
                elseif iv==nvp
                    string_ = 'handover';
                else
                    string_ = '';
                end
                text(viapoint.T(1,4,iv)+0.05, viapoint.T(2,4,iv),viapoint.T(3,4,iv), string_);
            end
        end
    end

    mp.viaPoint.q    = q;
    mp.viaPoint.qdot = qdot;
    mp.viaPoint.indexes    = viapoint.idx;
    mp.viaPoint.T    = viapoint.T;

    % Override the previous orientations and update the robot init
    % =========================================================
    init2 = interpolate_poses(init, mp.viaPoint.indexes, mp.viaPoint.T);    
    
    dbg = 0;
    if dbg
        
        h_ =figurewe;
        tmp.hfig = h_;
        tmp.nMaxPlots   = 20;    
        tmp.axis_length = 0.05;
        tmp.pathPlotStyle = struct('LineWidth', 2, 'Color', 'r');
        tmp.axesPlotStyle{1} = struct('LineWidth', 2.5, 'Color', 'r');
        tmp.axesPlotStyle{2} = struct('LineWidth', 2.5, 'Color', 'g');
        tmp.axesPlotStyle{3} = struct('LineWidth', 2.5, 'Color', 'b');  
        homogTransfPlot( init.T, tmp );
        
        h_ =figurewe;
        tmp.hfig = h_;
        tmp.axis_length = 0.075;
        tmp.axesPlotStyle{1} = struct('LineWidth', 1.5, 'Color', 'r');
        tmp.axesPlotStyle{2} = struct('LineWidth', 1.5, 'Color', 'g');
        tmp.axesPlotStyle{3} = struct('LineWidth', 1.5, 'Color', 'b');           
        homogTransfPlot( init2.T, tmp );
        
        keyboard
    end
 
    % r2d(tr2rpy(mp.viaPoint.T(:,:,2)))
          
end

















