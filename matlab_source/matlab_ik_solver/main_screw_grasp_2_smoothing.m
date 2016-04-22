% load file
%    ./catkin_record_vrpn/src/IJRR/func_robot_vrep/darias_human_20160311_withIK.ttt
%
% This file will refine the trajectory
%
function main_shelf_2_smoothing()


    clear; clc; close all; dbstop if error;
    %clear classes;
    initialize_dropbox_path(1,0,1);
    addpath('../func_robot_vrep');
    addpath('../func_robot_vrep/func_vrep_bridge');

    ik = initialize_vrep_darias();

    [Tc, screwc] =  load_screw_data();    


    % new rest posture for getting screw on shelf
    ik.TrestPosture = [
        0.0335    0.8330    0.5522    0.3082
       -0.0298    0.5531   -0.8326   -0.5443
       -0.9990    0.0114    0.0434   -0.1951
             0         0         0    1.0000];
    ik.qRestPosture = [-0.7299   -1.6260    1.9364   -2.0335    1.5337   -1.1985    0.3269];


    %% run on each shelf

    if 0 % run IK on fine discretization. TURN IK ON in vrep
        load('DMP_10to10_coarse.mat');
        [dmpTmp, q] = collect_full_trajectories(ik, DMP, Tc, screwc);
        save('DMP_fine.mat', 'dmpTmp', 'q');
    else
        load('DMP_fine.mat');
    end

    if 1 % run FK on fine discretization. TURN IK OFF in vrep
        [sol_screw_grasp] = smooth_trajectories(ik, q, 0.000001);
        save('sol_screw_grasp.mat', 'sol_screw_grasp');
    end
    
end

function [sol] = smooth_trajectories(ik, qC, pauseTime)
    
    nDemo = numel(qC);
    
    indexDummyTip      = ik.getIndex('Dummy_tip');
    
    for d=nDemo:-1:1
        
        q = qC{d};
        
        for j=1:7
            qs(:,j) = smoothf(q(:,j));
        end
        figurew(['jointTraj_' num2str(j)]);
        qstmp = bsxfun(@minus,qs, qs(1,:)); 
        qtmp  = bsxfun(@minus,q, q(1,:));
        plot(qstmp, 'LineWidth', 2);
        plot(qtmp, 'LineWidth', 1);

        % play this trajectory in Darias (be sure to turn ik off)
        for t=1:numel(qs(:,1))
            ik.setJointAngles(qs(t,:));
            [~, ~, T] = ik.readGenericCoordinates(indexDummyTip);
            Ttraj(:,:,t) = T;
            pause(pauseTime)
        end
        
        sol{d}.T = Ttraj;
        sol{d}.q = qs;
        
    end
        
end

function [dmpTmp, q] = collect_full_trajectories(ik, DMP, Tc, shelfc)


    for k = 10:-1:1

        ik.backToRestPosture;
        T     = Tc{k};
        nTraj = size(T,3);

        % get good initial guess
        T = improve_initial_guess(T, shelfc{k}.endEffPos, ik);

        % defining via points
        vp.init = ik.TrestPosture;
        vp.finl = shelfc{k}.endEffPos;

        ik.sendTargetCartesianCoordinates(vp.finl(1:3,4), tr2rpy(vp.finl(:,:)), ik.getHandle('tool'), 1);

        h = createFig( [], squeeze(T(1:3,4,:)), [], vp);

        nUpdates = 10; nRollOut = 25;
        param.fixViaPointPoses   = 1;    % 1: is usually the standard use when the orientations 
                                         % are already given by the task. 0: means
                                         % that the orientation that results from
                                         % the ref. frame search will be used
                                         % instead.
        param.allowDMPShapeChange = 1;  % 1: DMP{k} weights are effective
        param.plotRollOuts=1;
        param.costWeight.similarity  = 1;
        param.costWeight.obstacle    = 0;
        param.costWeight.viaPoint    = 0;
        param.costWeight.viaPointMid = 0;
        param.costWeight.odometry    = 10000000;
        param.costWeight.IK          = 100;

        ik.nTraj = nTraj;

        % get trajectory
        [dmpTmp{k}, q{k}] = main_loop(ik, DMP{k},  h, nUpdates, nRollOut, param, 1);


        close all

    end 

end




