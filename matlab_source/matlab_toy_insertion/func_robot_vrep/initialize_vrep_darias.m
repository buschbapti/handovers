function robot =  initialize_vrep_darias()

    vrepObj = Vrep( );

    % Initialize Baxter
    robot = VrepDarias(vrepObj);
    %ik = Darias_ik(vrepObj);
    robot.simStart;
    robot.getDummyHandlesFromVREP({'Dummy_target', 'Dummy_tip', 'tool'});


    robot.requestStreaming; % this is important!


    % Update Baxter rest posture
    % ============================
    if 0
        [~, ~, restT, qRest] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
        save('dariasSLRest.mat', 'restT', 'qRest');
    else
        load('dariasSLRest.mat');
    end

    robot.qRestPosture = qRest; % robot.setJointAngles( qRest );
    robot.TrestPosture = restT;    
  

    % Pre-set robot commands 
    % ===========================================

    % Set arm to relaxed position instantaneously
    robot.backToRestPosture(); 


    % Set hand to relaxed position instantaneously
    robot.setFingersDefaultPos;
    
end