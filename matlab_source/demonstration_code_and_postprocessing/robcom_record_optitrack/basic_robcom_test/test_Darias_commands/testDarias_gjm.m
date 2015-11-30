function testDarias_gjm(ip, port)
%  testDarias_gjm '130.83.164.51'
%   testDarias_gjm
% darias streaming rate: set to 50 Hz again

global darias;


    ip = '130.83.164.51';

    % Helper function
    function tf = userChoice(question,default)
        if(default) 
            choices = 'Y/n';
            defChoice= 'Y';
        else
            choices = 'y/N';
            defChoice= 'N';
        end
        done = false;
        while(~done)
            choice = input(sprintf('%s %s: ',question,choices),'s');
            if(isempty(choice))
                choice = defChoice;
            end
            if(strcmpi(choice(1),'y'))
                done = true;
                tf = 1;
            elseif(strcmpi(choice(1),'n'))
                done = true;
                tf = 0;
            else
                fprintf('invalid input\n');
            end
        end
        
    end

    function printState(state)
        persistent t;
        
        if(isempty(t))
            t = tic();
        else
            toc(t);
            t = tic();
        end
        
        state.print();
        
    end

    if(~exist('ip','var') || isempty(ip))
        ip = '127.0.0.1';
    end
    
    if(~exist('port','var') || isempty(port))
        port = 2013;
    end

    fprintf('\n\nStarting Darias on ''%s:%d''\n',ip,port);
    darias = Darias(ip,port);
    darias.start();
    pause(1);

    if 1 % (userChoice('Test state callback?',true))
        darias.addCallback(@printState)
        pause(1)
        darias.callbacks = {};
    end
    
    
    fprintf('\n\nRequesting control\n');
    darias.requestControl();
    pause(1);

    
    fprintf('Setting streaming to 50 Hz\n\n');
    for k=1:20
        darias.setStreamingRate(50); % MUST do it to not get error
        pause(0.1);
    end
    pause(1);    
    
    darias.addCallback(@printState)
    pause(2)
    darias.callbacks = {};
    
    darias.setEndEffOffset(Darias.ENDEFF_RIGHT_ARM,[0.0543 0.0383 0.078+0.1833 0 0.5780 0]);
         

    if 0

        fprintf('\nRight fingers bending (5 sec) and stretching again (5 sec)\n');
        cmdFingerRight = robcom.commands.GotoJointCommand(Darias.GROUP_RIGHT_HAND);
        cmdFingerRight.addStates([5 5],[0.0 1.1 1.0 0.0 1.1 1.0 0.0 1.1 1.0 0.0 1.1 1.0 0.0 1.1 1.0 ; 0.0 0.2 0.2 0.0 0.2 0.2 0.0 0.2 0.2 0.0 0.2 0.2 0.0 0.2 0.2]);
        darias.sendCommand(cmdFingerRight);
        pause(10);
    end
    
    
    if 1 % (userChoice('Test GotoJoint command?',true))        
        
        fprintf('\nRight arm to zero position (10 sec) to a middle position (5 sec) and back to start position (5 sec)\n');
        cmdJointRight = robcom.commands.GotoJointCommand(Darias.GROUP_RIGHT_ARM,[],1);
        cmdJointRight.addStates([10 5 5],[0 0 0 0 0 0 0; -0.150 0.400 1.000 -1.000 1.000 -0.300 0.500; -0.300 0.900 2.000 -1.400 2.000 -0.500 1.000]);
        darias.sendCommand(cmdJointRight);
        pause(21);
    end
    
    
    if 1%(userChoice('Test GotoCart command?',true))
        fprintf('\nRight arm to center (10 sec), to intermediate position (5 sec) and back to start position (5 sec)\nWith vecolity at intermediate step\n');
        cmdCartRight = robcom.commands.GotoCartCommand(Darias.GROUP_RIGHT_ARM,[],1);
        cmdCartRight.addStates([10 5 5],[0.50 -0.10 -0.20 0.3542 -0.3621 0.8166 0.2776; 0.70 -0.30 0.15 0.3542 -0.3621 0.8166 0.2776; 0.653 -0.371 -0.245 0.221 -0.022 0.893 0.392]);

        darias.sendCommand(cmdCartRight);
        
        pause(21);
    end

    if 0 % (userChoice('Test end effector offset?',true))
        fprintf('\nSetting end effector offset for right arm to wrist\n');
        fprintf('Press any key to continue...\n');
        pause;
        darias.setEndEffOffset(Darias.ENDEFF_RIGHT_ARM,[0 0 0.0780 0 0 0]);

        fprintf('\nSetting end effector offset for right arm to default\n');
        fprintf('Press any key to continue...\n');
        pause;
        darias.setEndEffOffset(Darias.ENDEFF_RIGHT_ARM,[0.0543 0.0383 0.078+0.1833 0 0.5780 0]);
         
        fprintf('\nSetting end effector offset for left arm to wrist\n');
        fprintf('Press any key to continue...\n');
        pause;
        darias.setEndEffOffset(Darias.ENDEFF_LEFT_ARM,[0 0 0.0780 0 0 0]);

        fprintf('\nSetting end effector offset for left arm to default\n');
        fprintf('Press any key to continue...\n');
        pause;
        darias.setEndEffOffset(Darias.ENDEFF_LEFT_ARM,[0.0543 -0.0383 0.078+0.1833 0 0.5780 0]);
    end
    
    
    fprintf('\n\nStopping and deleteing Darias\n');

    darias.stop();
    darias.delete();
    
    if 0        
        recover_java_crash;        
    end

end















