% load file
%    ./catkin_record_vrpn/src/IJRR/func_robot_vrep/darias_human_20160311_withIK.ttt
%
% This file prepares the trajectories on a calibrated scene. It must run on
% a specific setting for a given shelf position. So I would calibrate it on
% the spot. And probably re-calibrate if the scene changes.
%
% This file is usually run with the IK disabled in vrep
%


clear; clc; close all; dbstop if error;
initialize_dropbox_path(1,0,1);

addpath('../func_robot_vrep');
addpath('../func_robot_vrep/func_vrep_bridge');
addpath('../gotos');
addpath('../../calibrate_optitrack_matlab/audio');

soundFiles = preloadSound();

robot = initialize_vrep_darias();

load('sol_screw_grasp.mat');
load('../screw_2deliver_on_table_optimize_isoemp/sol_screw_deliver.mat');
load('dariasSLRest.mat');

% new rest posture for getting screw on shelf
ik.TrestPosture = [
    0.0335    0.8330    0.5522    0.3082
   -0.0298    0.5531   -0.8326   -0.5443
   -0.9990    0.0114    0.0434   -0.1951
         0         0         0    1.0000];
ik.qRestPosture = [-0.7299   -1.6260    1.9364   -2.0335    1.5337   -1.1985    0.3269];

% final position to deliver the screw on the table
load('../../generic_robot_commands/screw_desired_joint_data/screw_coordinates_deliver_on_table_in_vrep.mat')


%% Create probabilistic DMPs to grasp screw on she lf first
if 0
    for j=1:7
        tmp=[];
        for k=1:numel(sol_screw_grasp) 
            tmp = [tmp sol_screw_grasp{k}.q(:,j)];
        end
        jointDataGraspScrew{j} = tmp;
    end

    % create probabilistic dmps
    for j=1:7 % condition each joint independently on the shelf position
        pdmpGraspScrew{j} = Probabilistic_DMP_shelf(jointDataGraspScrew{j}, 1);
    end
    save('pdmp_jointData_graspScrew.mat', 'jointDataGraspScrew', 'pdmpGraspScrew');
else
    load('pdmp_jointData_graspScrew.mat');
end

%% Create probabilistic DMPs to delive screw on table first
if 0
    for j=1:7
        tmp=[];
        for k=1:numel(sol_screw_deliver) 
        %for k=[1 2 3 4 5 6 7 9 10] % screw 8 was an outliar
            tmp = [tmp sol_screw_deliver{k}.q(:,j)];
        end
        jointDataDeliverScrew{j} = tmp;
    end
    
    % create probabilistic dmps
    for j=1:7 % condition each joint independently on the shelf position
        pdmpDeliverScrew{j} = Probabilistic_DMP_shelf(jointDataDeliverScrew{j}, 1);
    end
    save('pdmp_jointData_deliverScrew.mat', 'jointDataDeliverScrew', 'pdmpDeliverScrew');
else
    load('pdmp_jointData_deliverScrew.mat');
end

%% select the screw
screwPos = 9;

%% screw 1
if screwPos ==1
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])
    
    
    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib1 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib1.mat', 'calib1');
    else
        load('calib1.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib1.deliverOnTable = deliverPosOnTable;

        calibScrewTraj1 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib1, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj1.mat', 'calibScrewTraj1');
    else
        load('calibScrewTraj1.mat');
    end
    calibScrewTraj = calibScrewTraj1;
    timeGoBack = [5  5  3];
end

%% screw 2
if screwPos ==2
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib2 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib2.mat', 'calib2');
    else
        load('calib2.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib2.deliverOnTable = deliverPosOnTable;

        calibScrewTraj2 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib2, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj2.mat', 'calibScrewTraj2');
    else
        load('calibScrewTraj2.mat');
    end
    calibScrewTraj = calibScrewTraj2;
    timeGoBack = [5  5  3];
end

%% screw 3
if screwPos ==3
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib3 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib3.mat', 'calib3');
    else
        load('calib3.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib3.deliverOnTable = deliverPosOnTable;

        calibScrewTraj3 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib3, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj3.mat', 'calibScrewTraj3');
    else
        load('calibScrewTraj3.mat');
    end
    calibScrewTraj = calibScrewTraj3;
    timeGoBack = [5  5  3];
end

%% screw 4
if screwPos ==4
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib4 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib4.mat', 'calib4');
    else
        load('calib4.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib4.deliverOnTable = deliverPosOnTable;

        calibScrewTraj4 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib4, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj4.mat', 'calibScrewTraj4');
    else
        load('calibScrewTraj4.mat');
    end
    calibScrewTraj = calibScrewTraj4;
    timeGoBack = [5  5  3];
end

%% screw 5 

if screwPos ==5
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib5 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib5.mat', 'calib5');
    else
        load('calib5.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib5.deliverOnTable = deliverPosOnTable;

        calibScrewTraj5 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib5, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj5.mat', 'calibScrewTraj5');
    else
        load('calibScrewTraj5.mat');
    end
    calibScrewTraj = calibScrewTraj5;
    timeGoBack = [5  5  3];
end

%% screw 6 
if screwPos ==6    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib6 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib6.mat', 'calib6');
    else
        load('calib6.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib6.deliverOnTable = deliverPosOnTable;

        calibScrewTraj6 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib6, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj6.mat', 'calibScrewTraj6');
    else
        load('calibScrewTraj6.mat');
    end
    calibScrewTraj = calibScrewTraj6;
    timeGoBack = [5  5  3];
end

%% screw 7 
if screwPos ==7
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib7 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib7.mat', 'calib7');
    else
        load('calib7.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib7.deliverOnTable = deliverPosOnTable;

        calibScrewTraj7 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib7, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj7.mat', 'calibScrewTraj7');
    else
        load('calibScrewTraj7.mat');
    end
    calibScrewTraj = calibScrewTraj7;
    timeGoBack = [5  5  3];
end

%% screw 8 
if screwPos ==8
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib8 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib8.mat', 'calib8');
    else
        load('calib8.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib8.deliverOnTable = deliverPosOnTable;

        calibScrewTraj8 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib8, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj8.mat', 'calibScrewTraj8');
    else
        load('calibScrewTraj8.mat');
    end
    calibScrewTraj = calibScrewTraj8;
    timeGoBack = [5  5  3];
end


%% screw 9 
if screwPos ==9
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib9 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib9.mat', 'calib9');
    else
        load('calib9.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib9.deliverOnTable = deliverPosOnTable;

        calibScrewTraj9 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib9, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj9.mat', 'calibScrewTraj9');
    else
        load('calibScrewTraj9.mat');
    end
    calibScrewTraj = calibScrewTraj9;
    timeGoBack = [5  5  3];
end


%% screw 10 
if screwPos ==10
    
    % fine tuned positon to gras the screw. You may need to calibrate this
    % often
    load(['../fine_tune_endEffector_using_KT/screwPositionOnShelf20160325/screw' num2str(screwPos) '.mat'])

    if 0  % tune each screw position individually. Turn IK ON
        adjust{screwPos}.approach_Z_hand   = -0.03; % how much the hand advances to grasp the screw on the hand coordinates
        adjust{screwPos}.approach_Z_world  =  0.015; % how much the and comes from the vertical direction while approaching the grasp of the screw
        adjust{screwPos}.approach_nTraj    = 25;
        adjust{screwPos}.fineCalibration = calibrationPos;
        calib10 = calibrate_screw_on_shelf(robot, sol_screw_grasp{screwPos}.T(:,:,end), sol_screw_grasp{screwPos}.q(end,:),  adjust{screwPos}); % IK must be ON
        save('calib10.mat', 'calib10');
    else
        load('calib10.mat');
    end
    if 0 % generate trajectories to get joint trajectroies. turn IK OFF
        paramhandover.qfinalPos = [];
        paramhandover.percentOfTrajectoryReturn = 0.35;
        calib10.deliverOnTable = deliverPosOnTable;

        calibScrewTraj10 = generate_calibrated_trajectories_approach_and_grasp(robot,...
                                       pdmpGraspScrew, jointDataGraspScrew, ...
                                       pdmpDeliverScrew, jointDataDeliverScrew, ...
                                       calib10, screwPos, paramhandover); % IK must be OFF!!
        save('calibScrewTraj10.mat', 'calibScrewTraj10');
    else
        load('calibScrewTraj10.mat');
    end
    calibScrewTraj = calibScrewTraj10;
    timeGoBack = [5  5  3];
end



%% run real robot
%timeGoBack = [15 15 3];
    
dariasIP = '130.83.160.228'; % asus
dariasIP = '130.83.164.68'; % bohr
dariasIP = '192.168.1.3'; % home (changes)

robotData = execute_real_robot_screw_wrap(calibScrewTraj, timeGoBack, dariasIP, soundFiles);











break


clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);

addpath('../func_robot_vrep');
addpath('../func_robot_vrep/func_vrep_bridge');

ik = initialize_vrep_darias();

load('sol_shelf.mat');
load('dariasSLRest.mat');

% Define this position to handover the plate to the human
ThandoverPlate = [
   -0.0000    0.8660    0.5000    0.83870
    0.0000   -0.5000    0.8660   -0.00
    1.0000    0.0000   -0.0000    0.0400
         0         0         0    1.0000];
qhandoverPlate = [
    1.1290   -0.8824    0.4408   -0.8917    2.9322    0.3763    2.9322];

if 0
    ik.sendTargetCartesianCoordinates(ThandoverPlate(1:3,4), tr2rpy(ThandoverPlate), ik.getHandle('Dummy_target'), 1);
    ik.setJointAngles(qhandoverPlate);
    [~, ~, ~, qhandoverPlate] = ik.readGenericCoordinates(ik.getIndex('Dummy_tip')');
end


% compute the grasping of the shelf
if 1 % turn IK on
   graspDepth = 0.1;
   for k=1:7
        ik.sendTargetCartesianCoordinates(sol_shelf{k}.T(1:3,4, end), tr2rpy(sol_shelf{k}.T(:,:, end)), ik.getHandle('Dummy_target'), 1);
        ik.setJointAngles(sol_shelf{k}.q(end,:));
        
        if k==1 % for each position I have to do a fine tuning here
            sol_shelf{k}.grasp.T = sol_shelf{k}.T(:,:, end)*se3(0,d2r(+15),0, 0,0, graspDepth);
            dz = -sol_shelf{k}.grasp.T(3,4)+sol_shelf{k}.T(3,4, end);
            sol_shelf{k}.grasp.T = se3(0,0,0, 0,0,dz)*sol_shelf{k}.grasp.T;
        end
        if k==2 % for each position I have to do a fine tuning here
            sol_shelf{k}.grasp.T = sol_shelf{k}.T(:,:, end)*se3(0,d2r(+15),0, 0,0,graspDepth);
            dz = -sol_shelf{k}.grasp.T(3,4)+sol_shelf{k}.T(3,4, end);
            sol_shelf{k}.grasp.T = se3(0,0,0, 0,0,dz)*sol_shelf{k}.grasp.T;
        end
        if k==3 % for each position I have to do a fine tuning here
            sol_shelf{k}.grasp.T = sol_shelf{k}.T(:,:, end)*se3(0,d2r(+15),0, 0,0,graspDepth);
            dz = -sol_shelf{k}.grasp.T(3,4)+sol_shelf{k}.T(3,4, end);
            sol_shelf{k}.grasp.T = se3(0,0,0, 0,0,dz)*sol_shelf{k}.grasp.T;
        end
        if k==4 % for each position I have to do a fine tuning here
            sol_shelf{k}.grasp.T = sol_shelf{k}.T(:,:, end)*se3(0,d2r(+0),0, 0,0,graspDepth);
            dz = -sol_shelf{k}.grasp.T(3,4)+sol_shelf{k}.T(3,4, end);
            sol_shelf{k}.grasp.T = se3(0,0,0, 0,0,dz)*sol_shelf{k}.grasp.T;
        end
        if k==5 % for each position I have to do a fine tuning here
            sol_shelf{k}.grasp.T = sol_shelf{k}.T(:,:, end)*se3(0,d2r(0),0, 0,0,graspDepth);
            dz = -sol_shelf{k}.grasp.T(3,4)+sol_shelf{k}.T(3,4, end);
            sol_shelf{k}.grasp.T = se3(0,0,0, 0,0,dz)*sol_shelf{k}.grasp.T;
        end
        if k==6 % for each position I have to do a fine tuning here
            sol_shelf{k}.grasp.T = sol_shelf{k}.T(:,:, end)*se3(0,d2r(+0),0, 0,0,graspDepth);
            dz = -sol_shelf{k}.grasp.T(3,4)+sol_shelf{k}.T(3,4, end);
            sol_shelf{k}.grasp.T = se3(0,0,0, 0,0,dz)*sol_shelf{k}.grasp.T;
        end
        if k==7 % for each position I have to do a fine tuning here
            sol_shelf{k}.grasp.T = sol_shelf{k}.T(:,:, end)*se3(0,d2r(+0),0, 0,0,graspDepth);
            dz = -sol_shelf{k}.grasp.T(3,4)+sol_shelf{k}.T(3,4, end);
            sol_shelf{k}.grasp.T = se3(0,0,0, 0,0,dz)*sol_shelf{k}.grasp.T;
        end        
        ik.sendTargetCartesianCoordinates(sol_shelf{k}.grasp.T(1:3,4, end), tr2rpy(sol_shelf{k}.grasp.T(:,:, end)), ik.getHandle('Dummy_target'), 1);        
        [~, ~, ~, sol_shelf{k}.grasp.q] = ik.readGenericCoordinates(ik.getIndex('Dummy_tip')');
   end
   save('sol_shelf_calibA.mat', 'sol_shelf');
else
   load('sol_shelf_calibA.mat'); 
end

% compute the trajectory in join space
if 1 % turn IK off
    for k=1:7
        clear qConnect
        for j=1:7 % the other half of the trajectory is given by the connection 
            sol_shelf{k}.grasp.qTraj(:,j) = goToTrapez( sol_shelf{k}.q(end,j), sol_shelf{k}.grasp.q(j), 10, []);
        end

        for t = 1:numel(sol_shelf{k}.grasp.qTraj(:,1))
            ik.setJointAngles(sol_shelf{k}.grasp.qTraj(t,:));
            pause(0.15);
        end
        sol_shelf{k}.grasp.qTrajRev = sol_shelf{k}.grasp.qTraj(end:-1:1,:);
        for t = 1:numel(sol_shelf{k}.grasp.qTraj(:,1))
            ik.setJointAngles(sol_shelf{k}.grasp.qTrajRev(t,:));
            pause(0.15);
        end  
    end
    save('sol_shelf_calibB.mat', 'sol_shelf');
else
    load('sol_shelf_calibB.mat');
end


% compute the return trajectory
if 1 % turn IK off
    for k=1:7

        ik.sendTargetCartesianCoordinates(sol_shelf{k}.T(1:3,4, end), tr2rpy(sol_shelf{k}.T(:,:, end)), ik.getHandle('Dummy_target'), 1);
        ik.setJointAngles(sol_shelf{k}.q(end,:));

        nTraj = numel(sol_shelf{k}.q(:,1));

        % reverse the trajectory
        sol_shelf{k}.qRev = sol_shelf{k}.q(end:-1:1,:);
        sol_shelf{k}.qRev = sol_shelf{k}.qRev(1:round(nTraj*1/2),:); % use only the first half of the reverse

        clear qConnect
        for j=1:7 % the other half of the trajectory is given by the connection 
            qConnect(:,j) = goToTrapez(sol_shelf{k}.qRev(end,j), qhandoverPlate(j), round(nTraj*1/2), []);
        end

        sol_shelf{k}.qRev = [sol_shelf{k}.qRev; qConnect];
        for j=1:7 % smooth
            sol_shelf{k}.qRev(:,j) = smoothf(sol_shelf{k}.qRev(:,j));
        end

        % animate to check if it is okay
        for t = 1:round(numel(sol_shelf{k}.qRev(:,1))*(1))
            ik.setJointAngles(sol_shelf{k}.qRev(t,:));
            pause(0.01)
        end

        figurew(['shelfPos_' num2str(k)]);
        plot(bsxfun(@minus, sol_shelf{k}.qRev, sol_shelf{k}.qRev(1,:)));
    end
    save('sol_shelf_calibC.mat', 'sol_shelf');
else
    load('sol_shelf_calibC.mat');
end
