function [out, hCurves] = homogTransfPlot( T, param )
% [To1, hCurves] = homogTransfPlot( T, param )
%
% INPUT
%   T: [4 X 4 X nT] path of transformation matrix
%
%   param.hfig: is a handle for a figure. If the field does not exist a new
%               figure will be created
%
%   param.axis_length: the length of the axes of the reference frame. You
%                      have to adjust this value depending on the scale of
%                      the path you are plotting.
%
%   param.nMaxPlots:   Do not plot one ref. frame for each time step, but
%                      rather evenly space their plots to the number of
%                      nMaxPlots.
%
%   param.pathPlotStyle: optional and used to define your own style to plot the path
%       Example: param.pathPlotStyle = struct('LineWidth', 0.1, 'Color', 'k', 'Marker', '.')
%
%   param.axesPlotStyle: optional and used to define your own style to plot the reference frame
%       along the trajectories. All axes will have the same color.
%       Example: param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [.5 .5 .5]);
%                param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 .5 .5]);
%                param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 .5]);
%
%   param.shadowHeight = scalar: plot the shadow of the trajectory on the
%                                XY plane. If the field does not exist no
%                                shadow will be plot.
%
% copy-paste example
% param.hfig = h1;
% param.axis_length = 0.5;
% param.nMaxPlots = 20;
% param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
% param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .5 .5]);
% param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 1 .5]);
% param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 1]);
% param.shadowHeight     = -1;
%


    hCurves = [];
    
    ul = param.axis_length;

    % define the XYZ reference frames
    refFrame(1).axis  = [ul 0 0 1]';
    refFrame(1).style = sty('r', [], 3);
    refFrame(2).axis  = [0 ul 0 1]';
    refFrame(2).style = sty('g', [], 3);
    refFrame(3).axis  = [0 0 ul 1]';
    refFrame(3).style = sty('b', [], 3);
    
    if ~isfield(param, 'axesPlotStyle')
        refFrame(1).style = sty('r', [], 1);
        refFrame(2).style = sty('g', [], 1);
        refFrame(3).style = sty('b', [], 1);        
    else
        refFrame(1).style = param.axesPlotStyle{1};
        refFrame(2).style = param.axesPlotStyle{2};
        refFrame(3).style = param.axesPlotStyle{3};
    end

    nT = size(T,3);

    % plot the path first
    % ..............................................
    if 0 % trying to improve here
        for k=1:nT
            p(k,:) = T(1:3, 4,k)';
        end
    else
        p = squeeze(T(1:3, 4, :))';
    end
    
    if ~isfield(param, 'hfig')
        figurew('rotations_along_path'); axis 'equal';
    else
        figure(param.hfig);
    end
    
    if ~isfield(param, 'pathPlotStyle')
        hCurves = [hCurves plot3(p(:,1), p(:,2), p(:,3), styw([0.7 0.7 0.7], 'o', [], '-', 5)  )];
    else
        hCurves = [hCurves  plot3(p(:,1), p(:,2), p(:,3), param.pathPlotStyle )];
    end
    
    if isfield(param, 'shadowHeight')
        
        if numel(T(1,1,:)) > 1
            hCurves = [hCurves plot3(p(:,1), p(:,2), param.shadowHeight*ones(size(p(:,3))),...
                       styw([0.7 0.7 0.7], [], 2)  )];
        else
            hCurves = [hCurves plot3(p(:,1), p(:,2), param.shadowHeight*ones(size(p(:,3))),...
                       styw([0.7 0.7 0.7], 'x', 2, [], 10)  )];            
        end
    end
    

    % ................................................
    % 
    if nT ~= 1  % there are at least two points
        
        if nT > param.nMaxPlots
            dt = nT/param.nMaxPlots;
            dt = round(dt);
            kvec = floor(1:dt:nT);
        else
            kvec = 1:nT;
        end
        
        

        if kvec(1)~=1
            kvec(1) = 1;
        end
        
        if kvec(end)~=nT
            kvec(end+1) = nT;
        end
    else % for a single point
        kvec=1;
    end
    
    
    shiftZ = 0.005; % shift the height of the frame in the Z direction such that the plot looks better.
    for k = kvec
        tmp  = T(:,:,k)*[0 0 0 1]'; % get the position of the origin of the ref frame
                                    % which is the same as the position of
                                    % the point itself.
        orig = tmp(1:3)';
        for j=1:3
            out(j).val(1,:) = orig; 
            tmp = T(:,:,k)*refFrame(j).axis;
            out(j).val(2,:) = tmp(1:3)';
            hCurves = [hCurves  plot3(out(j).val(:,1), out(j).val(:,2), out(j).val(:,3)+shiftZ, refFrame(j).style )];
        end
    end

end





