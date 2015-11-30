

clear; clc; close all; dbstop if error;
initialize_dropbox_path(1, 1);

folder = 'interaction_data_plate_on_shelf';
%folder = 'interaction_data_plate_on_table';

folderNames = get_folder_names(['./' folder '/']);

h1 = figurewe; 
xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';
set_fig_position([0.486  0 0.485 0.934]); 
view([1 -1 1 ])
    
%% Step 1. Check all data here
for k=1:[]  %numel(folderNames)   

    (folderNames{k})
    if ~isempty( findstr(folderNames{k}, 'exp') )
        try
            load([folderNames{k} '/dataMocap.mat'  ]);
            plot_human_robot_cap(dataMocap, 0.07, 1);
        catch
            disp(['Couldnt read ', folderNames{k}]);
        end
       
    end
    
end



%% Step 2. Use this data set as demonstrations and also to get the via point

h2 = figurewe('Original_VS_ViaPoints'); 
xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';
set_fig_position([0.486  0 0.485 0.934]); 
view([1 -1 1 ])

load(['exp151104_163911_shelf4' '/dataMocap.mat'  ]);

% fix format of data
clear dataNew
dataNew.t = dataMocap.t;
dataNew.humanT = dataMocap.human.T;
dataNew.robotT = dataMocap.robot.T;
dataMocap = dataNew;

% filter noise
dataMocap.robotT = filter_T(dataMocap.robotT, 1);
dataMocap.humanT = filter_T(dataMocap.humanT, 1);

% creating a fake human demonstration based on the robot final position
% this is pure heuristic and created by visual inspection
dataMocap.humanT = eye(4);
dataMocap.humanT(1:3,4) = dataMocap.robotT(1:3,4,end); 
dataMocap.humanT(2,4) = dataMocap.humanT(2,4)-0.1;
T = my_trotx(d2r(-90)); 
dataMocap.humanT = dataMocap.humanT*T; 
T = my_trotz(d2r(90)); 
dataMocap.humanT = dataMocap.humanT*T; 
dataMocap.human  = dataMocap.humanT(1:3,4)';
plot_human_robot_cap(dataMocap, 0.1, 1);


naturalDemo.robot.T = dataMocap.robotT;
naturalDemo.robot.p = squeeze(dataMocap.robotT(1:3,4,:))';
naturalDemo.human.T = dataMocap.humanT;
naturalDemo.human.p = squeeze(dataMocap.humanT(1:3,4,:))';



%% Prepare data
figurew('RobotTrajRaw');
subplot(3,1,1); hold on; grid on;
plot( naturalDemo.robot.p(:,1), 'ro');
subplot(3,1,2); hold on; grid on;
plot( naturalDemo.robot.p(:,2), 'ro');
subplot(3,1,3); hold on; grid on;
plot( naturalDemo.robot.p(:,3), 'ro');

% cut the start and end of the trajectory and plot the final stuff
idxStart = 90;
idxEnd   = numel(naturalDemo.robot.T(1,1,:));
naturalDemo.robot.p = naturalDemo.robot.p(idxStart:idxEnd,:);
naturalDemo.robot.T = naturalDemo.robot.T(:,:,idxStart:idxEnd);
figurew('RobotTrajCut');
subplot(3,1,1); hold on; grid on;
plot( naturalDemo.robot.p(:,1), 'bo');
subplot(3,1,2); hold on; grid on;
plot( naturalDemo.robot.p(:,2), 'bo');
subplot(3,1,3); hold on; grid on;
plot( naturalDemo.robot.p(:,3), 'bo');


idxViaPoint = 375; 

% by visual inspection I define where the via poin in the natural
% demonstration is
naturalDemo.viaPointIdx = idxViaPoint-5:idxViaPoint+5;
subplot(3,1,1); hold on; grid on;
plot( naturalDemo.viaPointIdx,  naturalDemo.robot.p(naturalDemo.viaPointIdx,1), 'ro');
subplot(3,1,2); hold on; grid on;
plot( naturalDemo.viaPointIdx, naturalDemo.robot.p(naturalDemo.viaPointIdx,2), 'ro');
subplot(3,1,3); hold on; grid on;
plot( naturalDemo.viaPointIdx, naturalDemo.robot.p(naturalDemo.viaPointIdx,3), 'ro');

naturalDemo.newViaPointGrasp.T = 'define from VREP or REBA';
naturalDemo.newViaPointGrasp.p = 'define from VREP or REBA';

% transpose here for compatibility with old code
naturalDemo.robot.p = naturalDemo.robot.p';
naturalDemo.human.p = naturalDemo.human.p';

param.hfig = figurewe('final');
param.axis_length = 0.05;
param.nMaxPlots = 20;
param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'k', 'Marker', 'none');
param.axesPlotStyle{1} = struct('LineWidth', 2, 'Color', [1 .005 .005]);
param.axesPlotStyle{2} = struct('LineWidth', 2, 'Color', [.005 1 .005]);
param.axesPlotStyle{3} = struct('LineWidth', 2, 'Color', [.005 .005 1]);
param.shadowHeight     = -1;
homogTransfPlot(naturalDemo.robot.T, param);

param.nMaxPlots = 20;
param.axis_length = 0.075;
param.pathPlotStyle    = struct('LineWidth', 1, 'Color', 'r', 'Marker', 'none');
param.axesPlotStyle{1} = struct('LineWidth', 4, 'Color', lightRGB(1));
param.axesPlotStyle{2} = struct('LineWidth', 4, 'Color', lightRGB(2));
param.axesPlotStyle{3} = struct('LineWidth', 4, 'Color', lightRGB(3));
homogTransfPlot(naturalDemo.human.T, param);


break


save(['./' folder '/post_processed_data/set1.mat'], 'naturalDemo');





















