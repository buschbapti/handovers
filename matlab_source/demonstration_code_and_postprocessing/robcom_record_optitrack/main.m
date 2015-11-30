% This is the experiment that was done with Doro
%
% The human grasps something on the shelf, while the other receives it
%
%

clear; clc; close all; dbstop if error;
initialize();

recover_java_crash;
     
global optitrack
port = 1511;
optitrack = OptiTrack(port);
add  = [2   -1   -2.5];
deg_ = [-45+add(1)  add(2)  add(3)];
newQ = [0.7071     d2r(deg_)  ];
newQ = newQ./norm(newQ);
optitrack.setTransform([-0.0349 0.0 0.0583  newQ]);
optitrack.start();
soundList = preloadSound;


%% record trajectory from mocap
dataMocap = test_record_data(optitrack, soundList,  1);

scale=1;
for k=1:numel(dataMocap.names)       
    for t=1:numel(dataMocap.t)        
        xyz   = scale*dataMocap.(dataMocap.names{k})(t,1:3) + [0 0 0  ]; % correct measurement
        quatOjb = Quaternion( dataMocap.(dataMocap.names{k})(t,4:end));
        T = [quatOjb.R xyz'; [0 0 0 1] ];              
        % rotate to match vrep configuration     
        if strcmp(dataMocap.names{k}, 'robot')
            in.axes = {'xdeg', 'ydeg'};
            in.val  = [93, -5];
            R_ = rotationSequential( quatOjb.R , in );
            T = [R_  xyz';  [0 0 0 1] ];            
        end         
        if strcmp(dataMocap.names{k}, 'human')  
            in.axes = {'x', 'zdeg', 'ydeg'};
            in.val  = [pi/2, 4, -3];
            R_ = rotationSequential( quatOjb.R , in );
            T = [R_  xyz';  [0 0 0 1] ];     
        end
        if strcmp(dataMocap.names{k}, 'cap')  
            in.axes = {'x'};
            in.val  = [-pi/2];
            R_ = rotationSequential( quatOjb.R , in );
            T = [R_  xyz';  [0 0 0 1] ];     
        end        
        dataMocap.([dataMocap.names{k} 'T'])(:,:,t) = T;
        if t==1
            text(xyz(1), xyz(2), xyz(3), dataMocap.names{k}  );
        end
    end    
end

%figurew; xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';
param.hfig = gcf;
param.axis_length = scale*0.15;
param.nMaxPlots =  50;
param.pathPlotStyle =  styw([0.6 0.6 0.6], 'o', 0.25, [], 5);
homogTransfPlot(dataMocap.robotT, param);
homogTransfPlot(dataMocap.humanT, param);
axis equal
set_fig_position([0.486  0 0.485 0.934]); 
view([0 0 1 ])
text(dataMocap.robotT(1,4,1), dataMocap.robotT(2,4,1), dataMocap.robotT(3,4,1), 'robot' );
%text(dataMocap.humanT(1,4,1), dataMocap.humanT(2,4,1), dataMocap.humanT(3,4,1), 'human' );

view([1 -1 1 ])

%% shift marker to be inside the hand
if 0   % skip if you want to get raw data
    
    dataMocapNew.t = dataMocap.t;

    physicalMeasure = 0;
    if physicalMeasure % Use this to find the exacto angle to put in the parameter angleAroundY
        h_ =  figurewe('debugAngles'); view([1 -1 1]);
        angles = homogTransfMatrixProjectedAngles(dataMocap.robotT(:,:,k), h_ );
    end
    % you have to get this parameters by phisically measuring the offsets of
    % the marker on the back of the hand until it falls inside the hand
    paramMarker.robot.xyzShift      = [0.05 0 0.05]';
    paramMarker.robot.angleAroundY  = d2r(180-135);
    for k=1:numel(dataMocap.robotT(1,1,:))
        dataMocapNew.robot.T(:,:,k) = shift_marker_on_hand_center(...
                                  dataMocap.robotT(:,:,k), paramMarker.robot);
    end


    % Human marker needs better calibration!!!
    paramMarker.human.xyzShift      = [0.05 0 0.05]';
    paramMarker.human.angleAroundY  = d2r(180-135);
    for k=1:numel(dataMocap.humanT(1,1,:))
        dataMocapNew.human.T(:,:,k) = shift_marker_on_hand_center( ...
                                  dataMocap.humanT(:,:,k), paramMarker.robot);
    end


    %param.hfig = plot_workspace;     view([1 -1 1]);    
    param.hfig = figurewe('fe');     view([1 -1 1]);    
    param.axis_length = 0.095;
    param.nMaxPlots = 20;
    param.shadowHeight = -0.475;
    param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'b', 'Marker', 'none');
    param.axesPlotStyle{1} = struct('LineWidth', 4, 'Color', lightRGB(1));
    param.axesPlotStyle{2} = struct('LineWidth', 4, 'Color', lightRGB(2));
    param.axesPlotStyle{3} = struct('LineWidth', 4, 'Color', lightRGB(3));
    %homogTransfPlot(dataMocap.humanT, param);
    %homogTransfPlot(dataMocap.robotT, param);

    param.axis_length = 0.05;
    param.nMaxPlots = 20;
    param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'r', 'Marker', 'none');
    param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 0 0 ]);
    param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [0 1 0 ]);
    param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [0 0 1]);
    %homogTransfPlot(dataMocapNew.human.T, param);
    homogTransfPlot(dataMocapNew.robot.T, param);
    
    dataMocap = dataMocapNew; 
end

%%

an = input('save it? ');
if an   
    fname = ['exp' getTimeStamp];
    mkdir(fname );
    save(['./' fname '/dataMocap.mat'], 'dataMocap');
end





break



script_temp_replot_data(dataMocap, 1);






break

figurew; xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';
param.hfig = gcf;
homogTransfPlot(dataMocap.robotT, param);
homogTransfPlot(dataMocap.humanT, param);
axis equal
set_fig_position([0.486  0 0.485 0.934]); 
view([01 0 0 ])
text(dataMocap.robotT(1,4,1), dataMocap.robotT(2,4,1), dataMocap.robotT(3,4,1), 'robot' );


figurew; xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';
param.hfig = gcf;
homogTransfPlot(dataMocap.robotT, param);
homogTransfPlot(dataMocap.humanT, param);
axis equal
set_fig_position([0.486  0 0.485 0.934]); 
view([0  0  1])
text(dataMocap.robotT(1,4,1), dataMocap.robotT(2,4,1), dataMocap.robotT(3,4,1), 'robot' );


figurew; xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';
param.hfig = gcf;
homogTransfPlot(dataMocap.robotT, param);
homogTransfPlot(dataMocap.humanT, param);
axis equal
set_fig_position([0.486  0 0.485 0.934]); 
view([0 -1 0 ])
text(dataMocap.robotT(1,4,1), dataMocap.robotT(2,4,1), dataMocap.robotT(3,4,1), 'robot' );















break







break





recover_java_crash;


break

% record human movement





% plot human movement
h.human = figurew('Human');












break


%% Tune parameters

%    repsCost.accel = 1; % value 1: makes it get stuck in achieving the
%    desired accel.

ikCost    = 0.25; % for kuka 0.25, for ur 0.25
mpCost    = 0;
nUpdates  = 5000;
nRollOuts = 5;
 
%% main code

% create a vrep connection with the robot
% restPostureNumber = 3;
% robot             = createRobotVREP(restPostureNumber);
robot  = createRobotVREP('Darias');


%% SELECT TYPE OF DEMO
if  0 % just put the trajectory on Darias frame
    mpCost = 0;
    ikCost = 6.5;
    trajectory_offset = [-0.63+0.0   -3.00015-0.00     0.1+0.0]; % meters (perturbed)    
    velGain=1;  
    
elseif 1 % conventional MP+IK
    mpCost    = 0;
    ikCost    = 6.5; 
    trajectory_offset = [-0.63   -3.00015     0.1]; % meters (close initial guess)    
    velGain   = 1.5;
    %velGain   = 0.5;
end

%%
% load training trajectory
[ProMP, init]  = load_golf_data_and_create_ProMP(0.01, 0, 100, trajectory_offset);
toolViaPoint   = get_tool_ref_frame(robot);

% Load general parameters
% =======================================
param.ikine    = userParamIKine(ikCost, 0.1, 0.00001);
param.reps     = userParamREPS(nUpdates, nRollOuts);
param.plot     = userParamPlot(param.reps.nUpdates);
param.repsCost = userParamCostREPS(init.p', init.pdot');

        
% main part
sys = sys_structure(ProMP, param); % create structure for ref frame parameters

pickPosition = 0;
param.mp = userParamMP(mpCost, init, pickPosition, toolViaPoint(1:3,4), [velGain velGain NaN]);

param.reps.Weight_IK_MP = [1  1]; % control the cost of each part
[~, computerName] = system('hostname');    

param.ikine.mask =  logical([ 1  1  1      0  0  0]);
k=1;

fileNameNow = my_time(['_' robot.name]);


[param, robot] = create_main_figures(param, robot, init, computerName, toolViaPoint(1:3,4));
initResampled = resample_for_IK(init, param);

[sys, currentT] = optimization_loop(sys, robot, initResampled, param);

saveContainer{k} =  prepare_container(ProMP, robot, sys, currentT, init, param, toolViaPoint, computerName, fileNameNow);    
save_and_load_solution(fileNameNow, 0, saveContainer{k});









break    

k=2;


    [param, robot, init, computerName, toolViaPoint, sys, ProMP] = save_and_load_solution(saveContainer{k-1}.fileNameNow, 1);
    %[param, robot, init, computerName, toolViaPoint, sys, ProMP] = save_and_load_solution('20150731_193436_LBR4', 1);
    
    close all;

    fileNameNow = my_time(['_' robot.name]);
    
    % returne parameters here
    param.reps.nUpdates = 50;
    param.ikine.createInitTraj.nSamples = 50;
    
    [param, robot] = create_main_figures(param, robot, init, computerName, toolViaPoint(1:3,4));
    initResampled = resample_for_IK(init, param);
    [sys, run_eval, currentT] = optimization_loop(sys, robot, initResampled, param);
    
    saveContainer{k} =  prepare_container(ProMP, robot, sys, currentT, init, param, toolViaPoint, computerName, fileNameNow);
    [saveContainerReloaded{k}, init2, initialSolT, finalSolT] = save_and_load_solution(my_time(['_' robot.name]), 0, saveContainer{k});
    
break
    
k=3;

    [param, robot, init, computerName, toolViaPoint, sys, ProMP] = save_and_load_solution(saveContainer{k-1}.fileNameNow, 1);
    close all;

    fileNameNow = my_time(['_' robot.name]);
    
    % returne parameters here
    param.reps.nUpdates = 50;
    param.ikine.createInitTraj.nSamples = 50;
    
    [param, robot] = create_main_figures(param, robot, init, computerName, toolViaPoint(1:3,4));
    initResampled = resample_for_IK(init, param);
    [sys, run_eval, currentT] = optimization_loop(sys, robot, initResampled, param);
    
    saveContainer{k} =  prepare_container(ProMP, robot, sys, currentT, init, param, toolViaPoint, computerName, fileNameNow);
    [saveContainerReloaded{k}, init2, initialSolT, finalSolT] = save_and_load_solution(my_time(['_' robot.name]), 0, saveContainer{k});
    
    
break

% restart next round
sys = saveContainerReloaded.sys;
robot = saveContainerReloaded.robot;
param = saveContainerReloaded.param;
[~, computerName] = system('hostname');
[param, robot] = create_main_figures(param, robot, init, computerName, toolViaPoint(1:3,4));
initR = resample_for_IK(finalSolT, param);
[sys, run_eval, currentT] = optimization_loop(sys, robot, initR, param);

clear saveContainer;
saveContainer.ProMP = ProMP;
saveContainer.robot = robot;
saveContainer.sys = sys;
saveContainer.run_eval = run_eval;
saveContainer.currentT = currentT;
saveContainer.init = init;
saveContainer.param = param;
[saveContainerReloaded2, ~, ~, finalSolT2] = save_and_load_solution(my_time(['_' robot.name]), 0, saveContainerReloaded);



sys = saveContainerReloaded2.sys;
robot = saveContainerReloaded2.robot;
param = saveContainerReloaded2.param;
[~, computerName] = system('hostname');
[param, robot] = create_main_figures(param, robot, init, computerName, toolViaPoint(1:3,4));
param.ikine.createInitTraj.nSamples=5;
initR = resample_for_IK(finalSolT2, param);
[sys, run_eval, currentT] = optimization_loop(sys, robot, initR, param);




























