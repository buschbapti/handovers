function [demo, viaPoint, hfig2] = load_interaction(demo, toolPosition, humanHandFinalLocation)
%  function ProMP = ProMP_wrapper(dt, scale)
    
    hfig2 = plot_workspace();    
       
    clear traj_ 
    load(demo.fileLocation);

    % for the REBA human, this has only one point as the final handover
    demo.human = naturalDemo.human;
    
    [robot, rate] = interpolateHomogTransf(naturalDemo.robot, 200);
    demo.robot = robot;
    
    % hacking: downsample via point
    demo.newVP.idx = unique(round(naturalDemo.viaPointIdx/rate));
   
    deleteSomePoints = 1;
    breakTraj = demo.newVP.idx(round(numel(demo.newVP.idx)/2));

    trajRange{1}.idx = deleteSomePoints:breakTraj-deleteSomePoints; % first half
    trajRange{2}.idx = breakTraj +1:numel(demo.robot.p(1,:)); % second half

    
    %% part 1 is easy as the via point of the robot is given by the object
    p=1;        

    %demo.part{p}.newVP.p = toolPosition(1:3,4);        
    demo.part{p}.robot.T = demo.robot.T(:,:,trajRange{p}.idx );
    demo.part{p}.robot.p = demo.robot.p(:,trajRange{p}.idx);        
    demo.part{p}.human.T = 'not in use';
    demo.part{p}.human.p = 'not in use';
    viaPoint.part{p}.T = toolPosition;
    viaPoint.part{p}.idx = numel( demo.part{p}.robot.p(1,:) );        

    %% part 2 needs some extra work
    % We have to specify the final handover position as a via point
    % To get this position I first see where the new handover position will
    % be from the final position of the REBA hand.
    p=2;
    demo.part{p}.robot.T = demo.robot.T(:,:,trajRange{p}.idx);
    demo.part{p}.robot.p = demo.robot.p(:,trajRange{p}.idx);
    demo.part{p}.human.T = demo.human.T;
    demo.part{p}.human.p = demo.human.p;    
    
    % Getting the robot via point (as the final handover position)
    %
    % 1. translation of the human hand in relation to origina demonstration
    xyTranslationOffset = humanHandFinalLocation(1:3,4) - demo.part{2}.human.T(1:3,4,end); 
    par_tmp.type = 'relative'; par_tmp.plot_flag = 0;    
    tmp.robot.T =  homogTransfTranslate(xyTranslationOffset, demo.part{2}.robot.T, par_tmp);
    
    dbg=0;
    if dbg
        param.hfig = plot_workspace();    
        param.axis_length = 0.15;
        param.nMaxPlots = 5;
        param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'b', 'Marker', 'o');
        param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .5 .5]);
        param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 1 .5]);
        param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 1]);
        param.shadowHeight     = -1.22;
        homogTransfPlot(demo.part{2}.robot.T, param);
        homogTransfPlot(tmp.robot.T, param);
    end
        
    % 2. Find the rotation angle
    if dbg
        h_ = param.hfig;   xlabel 'x'; ylabel 'y'; zlabel 'z';
    else
        h_=[];
    end
    angles1 = homogTransfMatrixProjectedAngles(humanHandFinalLocation,  h_ ); % REBA optimal location
    angles2 = homogTransfMatrixProjectedAngles(demo.part{2}.human.T(:,:,end), h_ ); % original demonstration
    deltaRPY = wrap2pi(angles1.vectorY.aroundWorldXYZ(3))-wrap2pi(angles2.vectorY.aroundWorldXYZ(3));
    zRotationOffset =1*wrap2pi(deltaRPY);
    tmpParam.center_of_rotation  = humanHandFinalLocation(1:3,4,1);  
    tmpParam.plot_flag = 0;
    tmp.robot.T = homogTransfRotate( [0 0 zRotationOffset], tmp.robot.T, tmpParam ); 

    if dbg
        param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'r', 'Marker', 'o');
        param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .5 .5]);
        param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 1 .5]);
        param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 1]);
        param.shadowHeight     = -1.22;
        homogTransfPlot(tmp.robot.T, param);
    end  

    % The new handover for the robot is given by a mirror on the human
    % REBA hand pose    
    viaPoint.part{p}.T = humanHandFinalLocation;
    
    % mirror the human handover as the robot via point
    % 1. rotate around Y axis 180
    viaPoint.part{p}.T = viaPoint.part{p}.T*my_trotx( d2r(180) );
    
    % 2. retract on the Z direction 20 cm
    viaPoint.part{p}.T = move_XYZ_on_intrinsic_frame(viaPoint.part{p}.T, [0 0 -0.1]');
    viaPoint.part{p}.idx  = numel(demo.part{p}.robot.T(1,1,:));    
    
    %demo.part{p}.newVP.p    = demo.part{p}.newVP.T(1:3,4,end);
    
    
    
%% Finally plot stuff here.
                        
    param.hfig = hfig2;
    param.axis_length = 0.05;
    param.nMaxPlots = 20;
    param.axesPlotStyle{1} = struct('LineWidth', 0.1, 'Color', [1 .5 .5]);
    param.axesPlotStyle{2} = struct('LineWidth', 0.1, 'Color', [.5 1 .5]);
    param.axesPlotStyle{3} = struct('LineWidth', 0.1, 'Color', [.5 .5 1]);
    param.shadowHeight = -1;
    param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'b', 'Marker', 'none');    
    homogTransfPlot(demo.part{1}.robot.T, param);
    param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'r', 'Marker', 'none');    
    homogTransfPlot(demo.part{2}.robot.T, param);
    homogTransfPlot(demo.part{2}.human.T, param);    
        
    param.axesPlotStyle{1} = struct('LineWidth', 4.1, 'Color', [1 .05 .05]);
    param.axesPlotStyle{2} = struct('LineWidth', 4.1, 'Color', [.05 1 .05]);
    param.axesPlotStyle{3} = struct('LineWidth', 4.1, 'Color', [.05 .05 1]);
    param.axis_length = 0.1;   
    param.pathPlotStyle    = struct('LineWidth', 2, 'Color', 'b', 'Marker', 'none');  

    % plot hew via points
    homogTransfPlot(viaPoint.part{1}.T, param);
    homogTransfPlot(viaPoint.part{2}.T, param);
    
    % 
    param.axesPlotStyle{1} = struct('LineWidth', 4.1, 'Color', lightRGB(1));
    param.axesPlotStyle{2} = struct('LineWidth', 4.1, 'Color', lightRGB(2));
    param.axesPlotStyle{3} = struct('LineWidth', 4.1, 'Color', lightRGB(3));    
    homogTransfPlot(humanHandFinalLocation, param);
    text(humanHandFinalLocation(1,4)+0.1, humanHandFinalLocation(2,4), ...
         humanHandFinalLocation(3,4), 'REBA');
end


function [out, dt] = interpolateHomogTransf(in, nTraj)

    nTrajOld = numel(in.p(1,:));
    
    dt = floor(nTrajOld/nTraj);
    ir = 1:dt:nTrajOld;
    
    out.p = in.p(:,ir);
    
    for j=1:numel(ir)
        out.T(:,:,j) = in.T(:,:,ir(j));
    end
    
end










