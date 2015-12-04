function [traj1_, traj2_] = get_robot_trajectory(posesMatlabFormat, robot, d_viaPoint, d_handover, humanA, ...
                            paramGeneral, lookupTraj1, lookupTraj2, soundPlayer)

    % 
    %[posesMatlabFormat] = adapt_positions(posesMatlabFormat, -30);
         
    [viaPoint, rebaHand ] = what_to_update(posesMatlabFormat, d_viaPoint, d_handover, humanA);        

    if ~isempty(viaPoint)
      %  play_sound(soundPlayer, 'step_one');

        traj1initGuess = search_lookupTable(lookupTraj1, viaPoint.T);

        % try decrease size to make FK faster
        traj1initGuess = resampleTraj(traj1initGuess, 50);

        % add grasping approach. 
        T_appr1.T = move_XYZ_on_intrinsic_frame(viaPoint.T, [traj1initGuess.vpAppr.xyzShift]*2);



        if paramGeneral.speedUpWithoutFKChecking
            traj1dmpConnect = fast_connect_with_DMP_wrapper(robot, traj1initGuess.sol.q, robot.qRestPosture, T_appr1.T, paramGeneral.dmpExtendTimeFactor);
            addEnd = robot.goTo(T_appr1.T, viaPoint.T, 20);
            robot.sendTargetCartesianCoordinates( T_appr1.T(1:3,4,end), tr2rpy(T_appr1.T(:,:,end)), robot.getHandle('Dummy_target'), 1);                                
        else
            paramFilterJoint.active=0;
            traj1dmpConnect = connect_with_DMP_wrapper(robot, [], robot.qRestPosture, T_appr1.T, paramFilterJoint, traj1initGuess.sol.q, paramGeneral.dmpExtendTimeFactor);
            addEnd = robot.goTo(traj1dmpConnect.T(:,:,end), viaPoint.T, 20);
            robot.sendTargetCartesianCoordinates( traj1dmpConnect.T(1:3,4,end), tr2rpy(traj1dmpConnect.T(:,:,end)), robot.getHandle('Dummy_target'), 1);                                
            traj1dmpWithGrasp.T = cat(3, traj1dmpConnect.T, addEnd);
        end

        % do approach to grasp IK here
        robot.setJointAngles( traj1dmpConnect.q(end,:))
        for t = 1:numel(addEnd(1,1,:))
            robot.sendTargetCartesianCoordinates( addEnd(1:3,4,t), tr2rpy(addEnd(:,:,t)), robot.getHandle('Dummy_target'), 1);
            [~, ~, ~, addEndQ(t,:)] = robot.readGenericCoordinates( robot.getIndex('Dummy_tip') );
        end

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


        if paramGeneral.speedUpWithoutFKChecking
            traj2dmpConnect = fast_connect_with_DMP_wrapper(robot, traj2initGuess.sol.q,T_appr1.T,  T_appr2.T, paramGeneral.dmpExtendTimeFactor);
            % add the start part that removes the object from its current position
            removeObject = robot.goTo(addEnd(:,:,end), T_appr1.T(:,:,1), 20);
        else            
            paramFilterJoint.active=0;
            traj2dmpConnect = connect_with_DMP_wrapper(robot, [], T_appr1.T, T_appr2.T, paramFilterJoint, traj2initGuess.sol.q);                
            removeObject = robot.goTo(traj1dmpWithGrasp.T(:,:,end), traj2dmpConnect.T(:,:,1), 20);                
            traj2dmpWithGrasp.T = cat(3, removeObject, traj2dmpConnect.T);
        end


        % do grasp to approach IK here
        %robot.sendTargetCartesianCoordinates(traj1dmpWithGrasp.T(1:3,4,end), tr2rpy(traj1dmpWithGrasp.T(:,:,end)), robot.getHandle('Dummy_target'), 1);
        robot.sendTargetCartesianCoordinates(addEnd(1:3,4,end), tr2rpy(addEnd(:,:,end)), robot.getHandle('Dummy_target'), 1);
        robot.setJointAngles( traj1dmpWithGrasp.q(end,:));

        for t = 1:numel(removeObject(1,1,:))
            robot.sendTargetCartesianCoordinates( removeObject(1:3,4,t), tr2rpy(removeObject(:,:,t)), robot.getHandle('Dummy_target'), 1);
            [~, ~, ~, removeObjectQ(t,:)] = robot.readGenericCoordinates( robot.getIndex('Dummy_tip') );
        end

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


    if paramGeneral.checkFinalSolution

        % Run the smoothed solution just for visual inspection purposes
        %robot.simStop;
        % do FK and also recover the homog. transf. matrix
        %   robot.setJointAngles(traj1.q(1,:),1);
        for k=1:numel(traj1.q(:,1))
            robot.setJointAngles(traj1.q(k,:),1);
            pause(0.015);
        end

        for k=1:numel(traj2.q(:,1))
            robot.setJointAngles(traj2.q(k,:),1);
            pause(0.015);
        end             
       keyboard

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

        if strcmp(humanA.agentName, 'humanAL')
            humanA.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), humanA.getHandle('Dummy_tip_humanA_L'), 1);    
        end
        if strcmp(humanA.agentName, 'humanAR')
            humanA.sendTargetCartesianCoordinates(rebaHand.T(1:3,4), tr2rpy(rebaHand.T), humanA.getHandle('Dummy_tip_humanA_R'), 1);    
        end
    end

end


