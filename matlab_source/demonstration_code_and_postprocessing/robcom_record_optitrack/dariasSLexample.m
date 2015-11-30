function KinestheticTeaching()
    % Kinesthetic teaching for the chair assemply.

    data = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
    collectData = 0;
    
    function getData(state)
        if(collectData)
            %darias.callbacks = {};
            %keyboard;
        
            data.timestamp(end+1,:) = state.timestamp;
            data.cart(end+1,:) = state.right.arm.msr.cart';
            data.joints(end+1,:) = state.right.arm.msr.joints.th';
        end
    end


    ip = '130.83.164.51';
    %ip = '127.0.0.1';
    port = 2013;
    
    dataDir = 'data/ChairKinest';
    dataName = 'kinest';

    darias = Darias(ip,port);
    
    darias.addCallback(@getData);
    
    darias.start();
    pause(0.25);
    darias.requestControl();
    darias.setStreamingRate(100);

    %darias.setEndEffOffset(Darias.ENDEFF_RIGHT_ARM,[0 0 0.0780 0 0 0]);
    darias.setEndEffOffset(Darias.ENDEFF_RIGHT_ARM,[0.0543 0.0383 0.078+0.1833 0 0.5780 0]);


    cmdJointRight = robcom.commands.GotoJointCommand(Darias.GROUP_RIGHT_ARM,[],1);
    %cmdJointRight.addStates([10],[-0.300 0.900 2.000 -1.400 2.000 -0.500 1.000]);
    %cmdJointRight.addStates([10],[-0.0300 0.500 2.000 -1.20 1.200 -0.050 2.500]);
    cmdJointRight.addStates([10],[-0.63 0.32 2.8 -0.666 0.540 -0.285 2.225]);
    
    cmdCartRight = robcom.commands.GotoCartCommand(Darias.GROUP_RIGHT_ARM);
    cmdCartRight.addStates([10],[0.70 -0.30 0.15 0.221 -0.022 0.893 0.392]);

    handClosed = [  0.0 1.1 1.1 ...
                    -0.2 1.1 1.1 ...
                    0.2 1.1 1.1 ...
                    0.0 1.1 1.0 ...
                    0.0 1.1 1.0];
 
    handOpened = [  0.0 0.15 0.09 ...
                    -0.2 0.15 0.09 ...
                    0.2 0.15 0.09 ...
                    0.0 1.1 1.0 ...
                    0.0 1.1 1.0];
    
    cmdHandClose = robcom.commands.GotoJointCommand(Darias.GROUP_RIGHT_HAND,[],1);
    cmdHandClose.addStates([1],handClosed);
    
    cmdHandOpen = robcom.commands.GotoJointCommand(Darias.GROUP_RIGHT_HAND,[],1);
    cmdHandOpen.addStates([1],handOpened);
    
    kinestTeaching = robcom.teachings.KinestheticTeaching(darias);
    
    recordingTime = 30;
    
    choice = [];
    teachingState=1;
    recordingState=1;
    teachingrecordingState=1;
    strOnOff = {'OFF','ON'};
    choicesStr={{'turn teaching on','turn teaching off','t'},{'start recording','stop recording','r'},{'turn teaching on and start recording','turn teaching off and stop recording','tr'}};
    while(isempty(choice) || ~strcmp(choice,'q') )
        
        str = sprintf('\n\nTeaching: %3s\tRecording:%3s\nChoices:\n',strOnOff{teachingState},strOnOff{recordingState});
        str = sprintf('%s\t%-4s: %s\n',str,choicesStr{1}{3},choicesStr{1}{teachingState});
        str = sprintf('%s\t%-4s: %s\n',str,choicesStr{2}{3},choicesStr{2}{recordingState});
        str = sprintf('%s\t%-4s: %s\n',str,choicesStr{3}{3},choicesStr{3}{teachingrecordingState});
        str = sprintf('%s\t%-4s: %s\n',str,'ti','timed recording');
        if(teachingState == 1)
            str = sprintf('%s\t%-4s: %s\n',str,'s','return to start position');
        end
        str = sprintf('%s\t%-4s: %s\n',str,'k','keyboard');
        
        str = sprintf('%s\t%-4s: %s\n',str,'ds','save data');
        str = sprintf('%s\t%-4s: %s\n',str,'dc','clear data');
        str = sprintf('%s\t%-4s: %s\n',str,'dsc','save and clear data');
        
        str = sprintf('%s\t%-4s: %s\n',str,'ho','open hand');
        str = sprintf('%s\t%-4s: %s\n',str,'hc','close hand');
        
        str = sprintf('%s\t%-4s: %s\n',str,'q','quit');
        
        choice = input(str,'s');
        
        switch choice
           case 't'
              if(teachingState == 1)
                kinestTeaching.activate();
                pause(0.25);
                kinestTeaching.start();
                teachingState = 2;
              else
                kinestTeaching.stop();
                pause(0.25);
                kinestTeaching.deactivate();
                teachingState = 1;
              end
           case 'r'
              if(recordingState == 1)
                collectData = 1;
                recordingState = 2;
              else
                collectData = 0;
                recordingState = 1;
              end
           case 'tr'
               if(teachingrecordingState == 1)
                collectData = 1;
                kinestTeaching.activate();
                kinestTeaching.start();
                teachingState = 2;
                recordingState = 2;
                teachingrecordingState = 2;
              else
                collectData = 0;
                kinestTeaching.stop();
                kinestTeaching.deactivate();
                teachingState = 1;
                recordingState = 1;
                teachingrecordingState = 1;
              end
           case 's'
               darias.sendCommand(cmdCartRight);
               darias.sendCommand(cmdJointRight);
           case 'k'
               keyboard;
           case 'ds'
               save(sprintf('%s/%s_%s',dataDir,dataName,datestr(now(),'yyyymmddHHMMSS')),'data');
           case 'dc'
               data = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
           case 'dsc'
               save(sprintf('%s/%s_%s',dataDir,dataName,datestr(now(),'yyyymmddHHMMSS')),'data');
               data = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
           case 'ti'
               recordingTimeChoice = input(sprintf('desired recording time [%.3f]: ',recordingTime));
               if(~isempty(recordingTimeChoice))
                   recordingTime = recordingTimeChoice;
               end
               
               collectData = 1;
               pause(recordingTime);               
               collectData = 0;
               
           case 'ho'
              if(teachingState == 1)
                darias.sendCommand(cmdHandOpen);
              else
                kinestTeaching.update(1,handOpened);
              end
               
           case 'hc'
               if(teachingState == 1)
                darias.sendCommand(cmdHandClose);
              else
                kinestTeaching.update(1,handClosed);
              end
               
           case 'q'
               fprintf('goodbye\n');
           otherwise
              fprintf(2,'Invalid Input!\n');
        end
        
    end
    
    save(sprintf('%s/%s_%s',dataDir,dataName,datestr(now(),'yyyymmddHHMMSS')),'data');
    
    darias.stop();
    darias.delete();
end

