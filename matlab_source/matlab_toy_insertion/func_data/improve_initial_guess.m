function T = improve_initial_guess(T, Tshelf, robot)
% This function uses as much information from the problem as possible to
% start the optimization with an initial guess. It first makes the end of
% the human trajectory (the grasping position of the plate on the shelf)
% to coincide with the position where the robot should grasp the plate.
% Then it will rotate the human demonstration such that the trajectory is
% kind of close to the expected final robot trajectory.
%

    param.hfig = cartesian_plot('better_init_guess');
    set_fig_position([0.558 0.0722 0.382 0.895]);
    plot_workspace(param.hfig);
    
    param.axis_length = 0.1;
    param.nMaxPlots = 20;
    param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k');
    param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 .0 .0]);
    param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [.0 1 .0]);
    param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [.0 .0 1]);
    param.shadowHeight     = -0.6;
    
    % get robot hand position
    [Thand] = robot.readEntityCoordinate('Dummy_tip');
    homogTransfPlot(Thand, param);
    homogTransfPlot(Tshelf, param);
    
    param.axis_length = 0.05; 
      
    
    dxyz = Tshelf(1:3,4)-T(1:3,4,end);
    
    for k=1:size(T,3)
        T(1:3,4,k) = T(1:3,4,k)+dxyz;
    end
    
    homogTransfPlot(T, param);
    

    
    % move them to world frame
    vec1 = [[T(1:2,4,end); 0]'; [T(1:2,4,1); 0]'];
    vec2 = [[T(1:2,4,end); 0]'; [Thand(1:2,4,1);0]'];    
   
    % compute angle between vectors at the origin
    vec1o = bsxfun(@minus, vec1, vec1(1,:));
    vec2o = bsxfun(@minus, vec2, vec2(1,:));
    
    plot(vec1o(:,1), vec1o(:,2))
    plot(vec2o(:,1), vec2o(:,2))
    
    yawAngleRad = atan2(norm(cross(vec1o(2,:),vec2o(2,:))),dot(vec1o(2,:),vec2o(2,:)));
    
    % check that rotation is positive or negative
    shift = T(1:3,4,end);
    Ttmp = T(:,:,1);
    Ttmp(1:3,4,1) = Ttmp(1:3,4,1)-shift;
    TtmpPos = se3(0, 0, yawAngleRad,0,0,0)*Ttmp;
    TtmpNeg = se3(0, 0, -yawAngleRad,0,0,0)*Ttmp;
    
    
    if 0% sum(TtmpPos(1:2,4)-Thand(1:2,4)).^2 > sum(TtmpNeg(1:2,4)-Thand(1:2,4)).^2
        yawAngleRad = -yawAngleRad;
        % BUG!!!! DOES NOT SEEM TO WORK IN THE SCREW CASE!!!!
    end   
    
    T_ = T;
    for k=1:size(T,3)
        T_(1:3,4,k) = T(1:3,4,k)-shift;
        Tpos(:,:,k) = se3(0, 0, yawAngleRad,0,0,0)*T_(:,:,k);
        Tpos(1:3,4,k) = Tpos(1:3,4,k) + shift;
    end
    homogTransfPlot(Tpos, param);
    
    
end




















