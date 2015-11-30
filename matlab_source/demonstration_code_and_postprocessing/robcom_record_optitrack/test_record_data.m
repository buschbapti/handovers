function dataMocap = test_record_data(optitrack,soundList,pauseTime)
    % Kinesthetic teaching for the chair assemply.

    
    dataSL = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
    mocapNames =[];
    dataMocap = [];
    myTimeS = 0;
    
    function getMocapNames( names, poses )       
        mocapNames = names;        
    end
     
    optitrack.addCallback(@getMocapNames);  pause(0.1);
    optitrack.callbacks = {};
    
    dataMocap = prepareMocapDataStructure(mocapNames);

    

    function getData(state)
        if(collectData)
            %darias.callbacks = {};
            %keyboard;
        
            dataSL.timestamp(end+1,:) = state.timestamp;
            dataSL.cart(end+1,:) = state.right.arm.msr.cart';
            dataSL.joints(end+1,:) = state.right.arm.msr.joints.th';
        end
    end


    function frequencyCheck( names, poses )
        persistent t1;
        if(isempty(t1))
            t1 = tic;
        else
            toc(t1);
            t1 = tic;
        end
    end

    function plotPose(pose, length, width, name)
        
        hold on;
        
        x(1,:) = quatrotate(quatconj(pose(1,4:7)),[length 0 0])+pose(1,1:3);
        y(1,:) = quatrotate(quatconj(pose(1,4:7)),[0 length 0])+pose(1,1:3);
        z(1,:) = quatrotate(quatconj(pose(1,4:7)),[0 0 length])+pose(1,1:3);

        plot3([pose(1) x(1)],[pose(2) x(2)],[pose(3) x(3)],'r', 'linewidth',width);
        plot3([pose(1) y(1)],[pose(2) y(2)],[pose(3) y(3)],'g', 'linewidth',width);
        plot3([pose(1) z(1)],[pose(2) z(2)],[pose(3) z(3)],'b', 'linewidth',width);

        if(exist('name','var') && ~isempty(name))
            text(pose(1),pose(2),pose(3),name);
        end
        
        
        
    end


    function plotFrame(names, poses, fig)
        
        persistent busy;
        if(isempty(busy))
            busy = false;
        end
        if(~busy)
            busy = true;            
            if ~isempty(fig)
                for idx = 1:numel(names);
                    plotPose(poses(idx,:), 50, 2, names{idx});
                end
            end
            
            dataMocap.t = [dataMocap.t;  toc(myTimeS) ];            
            for idx = 1:numel(names)
                dataMocap.(names{idx}) = [dataMocap.(names{idx}); poses(idx,:)];                
            end                   
            busy = false;
        end
    end



    function plotFrameOLD(names, poses, fig)
        
        persistent busy;
        if(isempty(busy))
            busy = false;
        end
        if(~busy)
            busy = true;
            
            %cla(fig);

            
            for idx = 1:numel(names);
                plotPose(poses(idx,:), 0.5, 2, names{idx});
            end
            
            busy = false;
        end
    end


    function printFrame( names, poses )
        
        fprintf('\nObjects:\n')
        
        for idx = 1:numel(names)
            fprintf('\t%s\t [',names{idx});
            fprintf(' %f',poses(idx,:));
            fprintf(' ]\n');
        end       
                
    end

    if 0

        fprintf('\n\nPrinting Frames\n');
        fprintf('Press any key to start...\nand afterwards any key to stop...\n');
        optitrack.addCallback(@printFrame);
        pause
        optitrack.callbacks = {};        
        
    end
    


    if 1       
    
        pause(1);
        % play sound to start
        
        h1 = figurew('frame');
        xlabel 'x (m)';  ylabel 'y (m)'; zlabel 'z (m)';
        axis equal;
        view([40 30]);
        grid on;
        plotPose([0 0 0 1 0 0 0], 0.15, 3, 'Optitrack');
       
        play_sound(soundList, 'beep');
        myTimeS = tic;
        %optitrack.addCallback(@(n,p)plotFrame(n,p, h1));
         optitrack.addCallback(@(n,p)plotFrame(n,p, []));
        fprintf('Press any key to stop...\n');
        if isempty(pauseTime)
            pause();
        else
            pause(pauseTime);
        end
        optitrack.callbacks = {};       
        
        axis equal;
        
        play_sound(soundList, 'beep');pause(0.25);
        play_sound(soundList, 'beep');
    
    end
    
    dataMocap

    
    


    
    return
    
    fprintf('\n\nChecking frequency\n');
    fprintf('Press any key to start...\nand afterwards any key to stop...\n');
    pause
    optitrack.addCallback(@frequencyCheck);
    pause
    optitrack.callbacks = {};
        
    
    fprintf('\n\nStopping OptiTrack');
    fprintf('Press any key to start...\n');
    pause
    optitrack.stop();

    
    fprintf('\n\nDeleting OptiTrack object');
    fprintf('Press any key to start...\n');
    pause
    optitrack.delete();

    
    while(isempty(choice) || ~strcmp(choice,'q') )
        
        if recordingState
            fprintf('Record state: %g \n', recordingState);    
        end
        
        fprintf('Record ON/OFF:  r\n');
        fprintf('Record OFF: t\n');
        fprintf('Save and clean: dsc\n');
        fprintf('keyboard: k\n');
        
        choice = input(' =>: ');
        
        switch choice
            case 'r'
              if(recordingState == 1)
                collectData = 1;
                recordingState = 2;
              else
                collectData = 0;
                recordingState = 1;
              end
    
           case 'k'
               keyboard;
               
           case 'dsc'
               
               save(sprintf('%s/%s_%s',dataDir,dataName,datestr(now(),'yyyymmddHHMMSS')),'dataSL');
               dataSL = struct('timestamp',zeros(0,1),'cart',zeros(0,7),'joints',zeros(0,7));
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
    
    save(sprintf('%s/%s_%s',dataDir,dataName,datestr(now(),'yyyymmddHHMMSS')),'dataSL');

end
















