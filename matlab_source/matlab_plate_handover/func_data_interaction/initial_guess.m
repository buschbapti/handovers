
function [init, frameShift] = initial_guess(roboRestT, demo, xyTranslationOffset, zRotationOffset, centerRotation, hworkspc, debug)

    dbg=0;
    
    % Check if human state is given or not
    if ischar(demo.human.T)
        useHuman = false;
    else
        useHuman = true;
    end

    % step 1: translate
    % =============================      
    par_tmp.type = 'relative';
    par_tmp.plot_flag = 0;
    
    if useHuman
        [init.human.T, init.human.p] =  homogTransfTranslate(xyTranslationOffset, demo.human.T, par_tmp);
    end
    [init.robot.T, init.robot.p] =  homogTransfTranslate(xyTranslationOffset, demo.robot.T, par_tmp);
    
    
    if dbg  
        param.hfig = gcf;
        param.axis_length = 0.15;
        param.nMaxPlots = 5;
        param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'b', 'Marker', 'o');
        param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .5 .5]);
        param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 1 .5]);
        param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 1]);
        param.shadowHeight     = -1.22;
        homogTransfPlot(init.human.T, param);
        homogTransfPlot(init.robot.T, param);
    end
    
    % step 2: rotate
    % =============================
    % now rotate the trajectory
    tmp.center_of_rotation = centerRotation;   
    tmp.plot_flag = 0;
    if useHuman
        [init.human.T, init.human.p] =  homogTransfRotate( [0 0 zRotationOffset], init.human.T, tmp );
    else
        init.human.T = 'not in use';  init.human.p = 'not in use';
    end
    [init.robot.T, init.robot.p] =  homogTransfRotate( [0 0 zRotationOffset], init.robot.T, tmp );    
    
    if dbg
        param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'r', 'Marker', 'o');
        if useHuman
            homogTransfPlot(init.human.T, param);    
        end
        homogTransfPlot(init.robot.T, param);
    end
                         
    % This is the transformation required to move the demonstration to the current human position.
    frameShift.static.xyz       = [xyTranslationOffset];
    frameShift.static.zRot      = [zRotationOffset];
    frameShift.static.zRotPivot = tmp.center_of_rotation;
    
    % This is the center of rotation that must also be perturbed durint REPS  
    % ==================================================================
    if 1 % Adding some heuristic to the problem:
         % Use the lat point which is the hand over position
        frameShift.dynamic.zRotPivotMoving = [init.robot.p(1,end); init.robot.p(2,end); init.robot.p(3,end)];
    end
                             
    % step 3: optional: plot and animate
    % =============================                 
    if debug(1)
        view([-1 -1 0.5])
        param.hfig = hworkspc;
        param.axis_length = 0.1;
        param.nMaxPlots = 20;
        param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'b', 'Marker', 'none');
        param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 0 0]);
        param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [0 1 0]);
        param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [0 0 1]);
        param.shadowHeight     = -1.22;
    
        if useHuman
            homogTransfPlot(init.human.T, param);
        end
        homogTransfPlot(init.robot.T, param);     
    end
    
    param.hfig = hworkspc;
    param.axis_length = 0.1;
    param.nMaxPlots = 20;
    param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'b', 'Marker', 'none');
    param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', lightRGB(1));
    param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', lightRGB(2));
    param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', lightRGB(3));
    param.shadowHeight     = -1.22;    
    homogTransfPlot(init.robot.T(:,:,1), param);   
    text(init.robot.T(1,4,1)+0.05, init.robot.T(2,4,1), init.robot.T(3,4,1), 'init' );
    plot3(init.robot.T(1,4,1), init.robot.T(2,4,1), param.shadowHeight , sty_nl([0.6 0.6 0.6], 'x', 2, [], 10) );
    
    homogTransfPlot(init.robot.T(:,:,end), param);   
    text(init.robot.T(1,4,end)+0.05, init.robot.T(2,4,end), init.robot.T(3,4,end), 'goal' );
    plot3(init.robot.T(1,4,end), init.robot.T(2,4,end), param.shadowHeight , sty_nl([0.6 0.6 0.6], 'x', 2, [], 10) );    
    
    homogTransfPlot(roboRestT, param);   
    text(roboRestT(1,4,1)+0.05, roboRestT(2,4,1), roboRestT(3,4,1), 'rest' );
    plot3(roboRestT(1,4), roboRestT(2,4), param.shadowHeight , sty_nl([0.6 0.6 0.6], 'x', 2, [], 10) );    
    
end



function  zRotationOffset = compute_angle_between_demonstration_and_currentHumanLocation(...
                    demoT, actual)
% here I have to compute the yawl angle (absolute world frame) between the angle of
% the human during the demonstration, and the current human angle.
% By human angle, we define the angle of the vector parallel to the ground,
% and pointing normal to the human chest. In other words, the angle at
% which the nose of the human is pointing.
% To get the angle of the demonstratin, I assume that the movement of the human hand 
% starts by moving normal to its chest, thus%, if I get the angle of the
% first velocity vector and project on the XY plane, I get the angle of the human
% during the demonstration.
%
% To get the current angle of the human I simply assume it as a given value
% that is, as a user input value.
% Ideally, I should be able to compute this angle, for example, by assuming
% that in the initial position, the palm of the human hand is pointing to
% his waist, that is, the z direction of the optritrack marker pointing -Z 
% (world frame). Then, his nose angle is
% simply the angle of the Y vector on the XY plane.

    % angle of the demonstration
    % ===================================    
    % Move the first displacement vector to the origin (only XY plane)
    nHeuristic = 5; % In theory should be 2, but if data is noisy better to skip
                    % some initial measurements.
    shiftVec = (demoT(1:2,4,nHeuristic)-demoT(1:2,4,1));

    zeroAngleVec = [1 0];
    
    absAngleDemo = angle_between_vec(shiftVec, zeroAngleVec);
    
    % fix range from [0 pi] to [-pi pi]
    absAngleDemo =fix_acos_angle(shiftVec(1), shiftVec(2), absAngleDemo);
    
    zRotationOffset = actual-absAngleDemo;
    
end

    
    
    