function [T1, gt, vpMid]=  load_data3D_arc(radius, zheight, nTraj)

    dbg=0;
    
    % lets create an arc
    xyz = make_arc(radius, nTraj);
    xyz(3,:) = zheight*ones(1,numel(xyz(3,:)));
    
if dbg
    h = figurewe('arc'); xlabel 'x';  ylabel 'y';  zlabel 'z';
    plot3(xyz(1,:), xyz(2,:), xyz(3,:));
end

    % creating a canonical trajectory where each state has the orientation
    % of the world frame
    for k =1:numel(xyz(1,:))
        T(:,:,k) = eye(4);
        T(1:3,4,k) = [xyz(1,k) xyz(2,k) xyz(3,k)]';
    end

    % add initial and final orientation orientation
    T(1:3,1:3,1)   = rpy2r([90 0 0]*pi/180)*T(1:3,1:3,1);
    T(1:3,1:3,end) = rpy2r([-90 -90 180]*pi/180)*T(1:3,1:3,end);
if dbg    
    homogTransfPlot(T);
end

    % now we interpolate between the initial and final orientations
    T = interpolate_poses(T, [1 numel(xyz(1,:))], T(:,:, [1 numel(xyz(1,:))]));
    
if dbg    
    param.hfig = h;
    param.axis_length      = 0.15;
    param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 0 0]);
    param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [0 1 0]);
    param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [0 0 1]);
    homogTransfPlot(T, param);
end

    % now I can move this trajectory to different locations and rotate it fully
    for k =1:numel(xyz(1,:))
        T1(:,:,k) = se3(0, 0 , -90*pi/180,  0.2, 0 ,0)*T(:,:,k);
    end

if dbg    
    homogTransfPlot(T1, param);
end

    gt=[];
    
    vpMid.xyz = [1 0.5 zheight; 0.2 -0.2 zheight];
    vpMid.idx = [1, nTraj];
    
    

end







