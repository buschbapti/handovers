function [traj1_, traj2_] = get_robot_trajectory(posesMatlabFormat, robot, d_viaPoint, d_handover, humanA, ...
                            paramGeneral, lookupTraj1, lookupTraj2, soundPlayer)

    
    if 0 % This is a fake robot trajectory generation
        load('20151104_160021_grasp_plate_on_table_ROS.mat');
        traj1_ = [traj1.t' traj1.q];
        traj2_ = [traj2.t' traj2.q];
        pause(5);        
    else        
        
        [viaPoint, rebaHand ] = what_to_update(posesMatlabFormat, d_viaPoint, d_handover, humanA);        
        
        fprintf('\n\n ** Paused **\n\n');
        fprintf('CHECK THAT THE SCENE WAS UPDATED ACCORDING TO THE\n');
        fprintf('DESIRED VIA POINTS AND HANDOVER POSITIONS\n');
        % pause();

        if ~isempty(viaPoint)
          %  play_sound(soundPlayer, 'step_one');
            
            traj1initGuess = search_lookupTable(lookupTraj1, viaPoint.T);
            
            % try decrease size to make FK faster
            traj1initGuess = resampleTraj(traj1initGuess, 50);
            
            % add grasping approach. 
            T_appr1.T = move_XYZ_on_intrinsic_frame(viaPoint.T, [traj1initGuess.vpAppr.xyzShift]*2);
            
            paramFilterJoint.active=0;
            traj1dmpConnect = connect_with_DMP_wrapper(robot, [], robot.qRestPosture, T_appr1.T, paramFilterJoint, traj1initGuess.sol.q, paramGeneral.dmpExtendTimeFactor);

            % reach the last state by a straight trajectory
            addEnd = robot.goTo(traj1dmpConnect.T(:,:,end), viaPoint.T, 20);
            
            % do approach to grasp IK here
            robot.sendTargetCartesianCoordinates( traj1dmpConnect.T(1:3,4,end), tr2rpy(traj1dmpConnect.T(:,:,end)), robot.getHandle('Dummy_target'), 1);
            robot.setJointAngles( traj1dmpConnect.q(end,:))
            for t = 1:numel(addEnd(1,1,:))
                robot.sendTargetCartesianCoordinates( addEnd(1:3,4,t), tr2rpy(addEnd(:,:,t)), robot.getHandle('Dummy_target'), 1);
                [~, ~, ~, addEndQ(t,:)] = robot.readGenericCoordinates( robot.getIndex('Dummy_tip') );
            end
            
            traj1dmpWithGrasp.T = cat(3, traj1dmpConnect.T, addEnd);
            traj1dmpWithGrasp.q = [traj1dmpConnect.q; addEndQ];

            d_viaPoint.sendTargetCartesianCoordinates(viaPoint.T(1:3,4), tr2rpy(viaPoint.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);    
 
           % save('currentViaPointSolution.mat', 'T_appr1', 'traj1dmpWithGrasp', 'viaPoint');
        else
           % load('currentViaPointSolution.mat');
        end        
       
        if ~isempty(rebaHand)
          %  play_sound(soundPlayer, 'step_two');
            
            % find the closes pre-computed solution in terms of the final position
            % of the trajectory
            traj2initGuess = search_lookupTable(lookupTraj2, rebaHand.T);            
            traj2initGuess = resampleTraj(traj2initGuess, 50);

            % add grasping approach. 
            T_appr2.T = rebaHand.T;
            T_appr2.T = T_appr2.T*my_trotx(pi); % mirror
            
            T_appr2.T = move_XYZ_on_intrinsic_frame(T_appr2.T, [0 0 -paramGeneral.offsetGripper_humanHand]');
            d_viaPoint.sendTargetCartesianCoordinates(T_appr2.T(1:3,4), tr2rpy(T_appr2.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);

            paramFilterJoint.active=0;
            traj2dmpConnect = connect_with_DMP_wrapper(robot, [], T_appr1.T, T_appr2.T, paramFilterJoint, traj2initGuess.sol.q);

            % add the start part that removes the object from its current position
            removeObject = robot.goTo(traj1dmpWithGrasp.T(:,:,end), traj2dmpConnect.T(:,:,1), 20);
            
            % do grasp to approach IK here
            robot.sendTargetCartesianCoordinates(traj1dmpWithGrasp.T(1:3,4,end), tr2rpy(traj1dmpWithGrasp.T(:,:,end)), robot.getHandle('Dummy_target'), 1);
            robot.setJointAngles( traj1dmpWithGrasp.q(end,:));
            
            for t = 1:numel(removeObject(1,1,:))
                robot.sendTargetCartesianCoordinates( removeObject(1:3,4,t), tr2rpy(removeObject(:,:,t)), robot.getHandle('Dummy_target'), 1);
                [~, ~, ~, removeObjectQ(t,:)] = robot.readGenericCoordinates( robot.getIndex('Dummy_tip') );
            end
                        
            traj2dmpWithGrasp.T = cat(3, removeObject, traj2dmpConnect.T);
            traj2dmpWithGrasp.q = [removeObjectQ; traj2dmpConnect.q];

        else
            error('The handover message says -999');
        end
        
        %% Prepare to send solution
        % play_sound(soundPlayer, 'step_three'); 
        
        d_viaPoint.sendTargetCartesianCoordinates(viaPoint.T(1:3,4), tr2rpy(viaPoint.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);

        load('../../config/baxterRest.mat'); robot.qRestPosture = qRest;  robot.TrestPosture = restT;  robot.backToRestPosture();

        paramGeneral.plotFlag = 0;        
        [traj1, traj2] = prepare_solution_for_real_experiment(robot, ...
                         traj1dmpWithGrasp, traj2dmpWithGrasp, paramGeneral.plotFlag);

        tFinal  = paramGeneral.tFinal;
        paramGeneral.nTraj = 150; 
        paramGeneral.filterOrder = 3; 
        paramGeneral.filterFreq = 0.1;

        traj1 = add_time_and_velocity(traj1, tFinal(1), paramGeneral);
        traj2 = add_time_and_velocity(traj2, tFinal(2), paramGeneral);

        traj1_ = [traj1.t' traj1.q];
        traj2_ = [traj2.t' traj2.q];        

        
        if 0% paramGeneral.checkFinalSolution
            play_sound(soundPlayer, 'do_you_want_to_see_final_sol');
            finalAns = input('Do you want to see the final solution? (0/1): ');
        end

       % play_sound(soundPlayer, 'trajectory_generated');
        
        
        if paramGeneral.checkFinalSolution
        
            % Run the smoothed solution just for visual inspection purposes
            %robot.simStop;
            % do FK and also recover the homog. transf. matrix
            for k=1:numel(traj1.q(:,1))
                robot.setJointAngles(traj1.q(k,:),1);
               % pause(0.1);
            end

            for k=1:numel(traj2.q(:,1))
                robot.setJointAngles(traj2.q(k,:),1);
                pause(0.1);
            end              
            %pause(3);
            %robot.simStart;
            
            keyboard
            
        end
        
        
        
        
        
    end       
    
end

function traj2 = resampleTraj(traj, nTraj)

    nTrajOrigi = numel(traj.sol.T(1,1,:));
    dt = nTrajOrigi/nTraj;
    
    if dt > 1 % resampling will decrease
        
        idx = 1:round(dt):nTrajOrigi;
        if idx(end) ~= nTrajOrigi
            idx(end+1) = nTrajOrigi;
        end
        
        if isfield(traj, 'vpAppr')
            traj2.vpAppr  = traj.vpAppr;
        end
        if isfield(traj, 'vpGrasp')
            traj2.vpGrasp = traj.vpGrasp;
        end       
        
        traj2.sol.T = traj.sol.T(:,:,idx);
        traj2.sol.q = traj.sol.q(idx,:);
        
    else % otherwise just return the original input
        traj2 = traj;
    end
    
    traj2.vpAppr  = traj.vpAppr;
    traj2.vpGrasp = traj.vpGrasp;
    
end

function [viaPoint, rebaHand ] = what_to_update(posesMatlabFormat, d_viaPoint, d_handover, humanA)

    if  sum(posesMatlabFormat(1,:)-(-999*ones(1,7)))==0 % execute via point update
        viaPoint=[];
    else
        viaPoint.T = fromQuaternionToHomog( posesMatlabFormat(1,:) );
        d_viaPoint.sendTargetCartesianCoordinates(viaPoint.T(1:3,4), tr2rpy(viaPoint.T), d_viaPoint.getHandle('Dummy_viaPoint_table'), 1);    
    end

    if  sum(posesMatlabFormat(2,:)-(-999*ones(1,7)))==0 % execute via point update
        rebaHand=[];
    else
        rebaHand.T = fromQuaternionToHomog(posesMatlabFormat(2,:));    
        d_handover.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), d_handover.getHandle('handoverPosition'), 1);
        humanA.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), humanA.getHandle('Dummy_tip_humanA_R'), 1);    
    end

end


