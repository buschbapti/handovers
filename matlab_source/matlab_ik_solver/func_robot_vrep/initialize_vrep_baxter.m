function robot =  initialize_vrep_baxter( startElbowConfig )

    vrepObj = Vrep( );

    % Initialize Baxter
    robot = VrepBaxter(vrepObj);
    robot.simStart;
    robot.getDummyHandlesFromVREP({'Dummy_target', 'Dummy_tip', 'Dummy_tipFK', 'handoverPosition'});

    robot.requestStreaming; % this is important!

    % Update Baxter rest posture
    % ============================
    if 0
        if 0        
            [qmid] = aux_useful_baxter_info();
            FKmode = 1;        
            robot.setJointAngles( d2r( qmid), FKmode );
            [restT] = robot.readEntityCoordinate('Dummy_tipFK');
            elbow.up.restT = restT;
            elbow.up.qRest = d2r(qmid);
        else
            %  [~, ~, elbow.down.restT, elbow.down.qRest] = ik.readGenericCoordinates(ik.getIndex('Dummy_tip'));
            [~, ~, elbow.down.restT, elbow.down.qRest] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
            save('elbowInitConfig.mat', 'elbow');
        end
        %[~, ~, restT, qRest] = robot.readGenericCoordinates(robot.getIndex('Dummy_tip'));
        %save('dariasSLRest.mat', 'restT', 'qRest');
    else
        % load('elbowInitConfig.mat'); <== this is done in the class 
    end
    robot.setRestPosture( startElbowConfig );

    % Pre-set robot commands 
    % ===========================================
    % Set arm to relaxed position instantaneously
    robot.backToRestPosture(); 

end