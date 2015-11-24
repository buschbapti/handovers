function dataRobot = execute_real_robot(dariasIp, soundFiles, vrepTrajCell, folderName)

    global darias;

    dataRobot = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
    collectDataRobot = 0;
    
    graspPlateOpenHand     = loadGotoSet({'openHand_plate_shelf.txt'},Darias.GROUP_RIGHT_HAND);
    graspPlateCloseHand    = loadGotoSet({'closeHand_plate_shelf.txt'},Darias.GROUP_RIGHT_HAND);
    cmdJointRightArmHome   = loadGotoSet({'startPosArm_slow.txt'},Darias.GROUP_RIGHT_ARM);
    
%%    
    function getDataRobot(state)
        if(collectDataRobot)
            disp('asdf')
            dataRobot.timestamp(end+1,:) = state.timestamp;
            dataRobot.cart(end+1,:) = state.right.arm.msr.cart';
            dataRobot.joints(end+1,:) = state.right.arm.msr.joints.th';
        end
    end



    function gotoCmd = loadGotoSet(filenames,GroupId)
        if(GroupId == Darias.GROUP_RIGHT_ARM)
            goto = NaN(0,8);
        else
            goto = NaN(0,16);
        end
        for idx = 1:numel(filenames)
            tmp = dlmread(['gotos/',filenames{idx}]);
            goto(end+(1:size(tmp,1)),:) = tmp;
        end
        
        gotoCmd = robcom.commands.GotoJointCommand(GroupId);
        gotoCmd.addStates(goto(:,1),goto(:,2:end));
    end  
    

    function gotoCmd = loadGotoSetVREP(vrepSol, GroupId, extraInitialTime)    
        
        gotoCmd = robcom.commands.GotoJointCommand(GroupId, [], 1);
        time = vrepSol.dt*ones(numel(vrepSol.q(:,1)),1);
        q    = vrepJointFixSign(vrepSol.q);
        
        if exist('extraInitialTime', 'var')
            time(1) = extraInitialTime;
        end
        gotoCmd.addStates(time, q);
        
    end


    function golf_old(vrepTraj)
        
        trajVREP = loadGotoSetVREP(vrepTraj, Darias.GROUP_RIGHT_ARM); 
        
        trajGOTOInit = robcom.commands.GotoJointCommand( Darias.GROUP_RIGHT_ARM);
        
        % add some point to point that lasts dT such that the arm can start
        % from the start of the ISOEMP solution
        trajGOTOInit.addStates(2.5, trajVREP.data(1,2:8) );   
                
        play_sound(soundFiles, 'initial_home_position');
        %darias.sendCommand(cmdJointRightArmHome); pause_cummulative(cmdJointRightArmHome);
        
        play_sound(soundFiles, 'beep');
        darias.sendCommand(trajGOTOInit);  pause_cummulative(trajGOTOInit);
       
        darias.sendCommand(trajVREP);      pause_cummulative(trajVREP);
       
    end
           
    function golf(vrepTraj)
        
        
        gotoCmd = robcom.commands.GotoJointCommand(Darias.GROUP_RIGHT_ARM, [], 0.5);
        time = vrepTraj.dt*ones(numel(vrepTraj.q(:,1)),1);
        q = vrepJointFixSign(vrepTraj.q);
        
        
        step1_goHomePos          = [5  cmdJointRightArmHome.data(2:end)];
        step2_goInitialGolfSwing = [3            q(1,:)];
        step3_swing              = [time(2:end)   q(2:end,:)];
        step4_goHomePos          = [5  cmdJointRightArmHome.data(2:end)];
        
        allSteps = [
                    step1_goHomePos
                    step2_goInitialGolfSwing
                    step3_swing
                    step4_goHomePos
                    ];
        
        gotoCmd.addStates(allSteps(:,1), allSteps(:,2:end));                
        darias.sendCommand(gotoCmd); pause_cummulative(gotoCmd);  
        
    end


%% main code starts here

    dariasPort = 2013;
    
    if isempty(dariasIp)
        dariasIp = '127.0.0.1';
    end
    
    darias = Darias(dariasIp,dariasPort);  darias.addCallback(@getDataRobot);
    darias.start(); darias.requestControl();
    
    kinestTeaching = robcom.teachings.KinestheticTeaching(darias);

    fprintf('Setting streaming to 50 Hz\n\n');
    for k=1:5
        fprintf('Setting streaming to 50 Hz\n\n');
        darias.setStreamingRate(50); % MUST do it to not get error
        darias.setEndEffOffset(Darias.ENDEFF_RIGHT_ARM,[0.0543 0.0383 0.078+0.1833 0 0.5780 0]);         
        pause(0.05);
    end

    kinestTeaching.stop(); pause(0.25); kinestTeaching.deactivate();
    statusKinestheticTeaching = false;
    
    
    %% execute first part of trajectory for grasping plate on shelf
    
    % Preparing stuff here =====================
    % ==========================================
    trajVREP_part1     = loadGotoSetVREP(vrepTrajCell{1}, Darias.GROUP_RIGHT_ARM, 3);  %))))))))))))))))))))))
    trajVREP_moveToGrasp     = loadGotoSetVREP(vrepTrajCell{2}, Darias.GROUP_RIGHT_ARM, 2);  %))))))))))))))))))))))
    trajVREP_part3     = loadGotoSetVREP(vrepTrajCell{3}, Darias.GROUP_RIGHT_ARM, 3);
    
    % add some point to point that lasts dT such that the arm can start
    % from the start of the ISOEMP solution and not from the true FRI home position
    dT = 5;                                                                            %))))))))))))))))))))))
    trajGOTOInit = robcom.commands.GotoJointCommand(Darias.GROUP_RIGHT_ARM);

    % execute first part ===========================
    % ========================================
    play_sound(soundFiles, 'initial_home_position');    
    darias.sendCommand(graspPlateOpenHand);
    darias.sendCommand(trajVREP_part1);  pause_cummulative(trajVREP_part1);
    
    % move towards the grasping
    darias.sendCommand(trajVREP_moveToGrasp);  pause_cummulative(trajVREP_moveToGrasp);
    
    
%keyboard
    darias.sendCommand(graspPlateCloseHand);  pause_cummulative(graspPlateCloseHand);

    
    
    darias.sendCommand(trajVREP_part3);  pause_cummulative(trajVREP_part3);
    trajGOTOInit.addStates(5, trajVREP_part1.data(1,2:8) );
    darias.sendCommand(graspPlateOpenHand);
    pause(1); % wait until the human pics the plate
    darias.sendCommand(trajGOTOInit);
        
    doit= false;
    while(doit)       
        
        fprintf('\n\n');
        fprintf('1: gravity comp. ON\n');
        fprintf('2: gravity comp. OFF\n');
        fprintf('5: home position\n');
        fprintf('e1: execute 1, ... N \n');
        fprintf('--------------------\n');
        fprintf('r: record start \n');
        fprintf('t: record stop \n');
        fprintf('--------------------\n');
        fprintf('z: open  hand plate \n');
        fprintf('x: close hand plate club\n');
        fprintf('--------------------\n');       
        fprintf('--------------------\n');
        fprintf('q: quit\n');

        userIn = input('Select an option: ', 's');
        
        switch userIn
            
            case 'e1'
                play_sound(soundFiles, 'beep');
                golf(vrepTrajCell{1});
            case 'e3'
                play_sound(soundFiles, 'beep');
                %golf(vrepTrajCell{3});
                golf_old(vrepTrajCell{3});
            case 'a' % 
                if statusKinestheticTeaching
                    fprintf('\n\n  KT\n\n');
                    kinestTeachingHandControl(kinestTeaching, graspScrewOpenHand_SHELF);
                else
                    fprintf('\n\n  noKT\n\n');
                    darias.sendCommand(graspScrewOpenHand_SHELF);
                end
            case 'z' % open hand for plate
                if statusKinestheticTeaching
                    fprintf('\n\n KT \n\n');
                    kinestTeachingHandControl(kinestTeaching, graspPlateOpenHand);
                else
                    fprintf('\n\n noKT \n\n');
                    darias.sendCommand(graspPlateOpenHand); 
                end
            case 'x' % close hand for plate
                if statusKinestheticTeaching
                    kinestTeachingHandControl(kinestTeaching, graspPlateCloseHand);
                else
                    darias.sendCommand(graspPlateCloseHand); 
                end
            case 'r'
                dataRobot = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
                collectDataRobot = 1;             
            case 't'
                collectDataRobot = 0; 
                fprintf('Size darias data: %g, av sampling %g (Hz)\n', numel(dataRobot.timestamp), 1/mean(diff(dataRobot.timestamp)))                
                time_stamp    = datestr(now(),'yyyymmdd_HHMMSS');
                saveRobotData(folder, time_stamp, dataRobot, 'txt'); 
                %saveRobotData(folder, time_stamp, dataRobot, 'mat');
                dataRobot = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
            case '1'
                kinestTeaching.activate();
                pause(0.25);
                kinestTeaching.start();
                statusKinestheticTeaching = true;
            case '2'
                kinestTeaching.stop(); pause(0.25); kinestTeaching.deactivate();
                statusKinestheticTeaching = false;
            case '5'
                play_sound(soundFiles, 'beep');
                darias.sendCommand(cmdJointRightArmHome); 
                darias.sendCommand(graspPlateOpenHand); 
                pause_cummulative(cmdJointRightArmHome, graspPlateOpenHand);
                statusDarias = 'home_position';                 
            case 'q'
                doit = 0;          
            otherwise
                play_sound(soundFiles, 'error');
                fprintf('\n\n unknown command.\n\n');
        end
        
        
    end
    
    

    
    darias.stop();
    darias.delete();
    darias = [];   
    

    if 1
        
        recover_java_crash;
        
    end
    
end


