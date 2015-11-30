function testOptiTrack( port )

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
            
            cla(fig);
            xlim([-0.5 1.5]);
            ylim([-1 1]);
            zlim([-1 1]);
            grid on;
            plotPose([0 0 0 1 0 0 0], 0.1, 1, 'Optitrack');
            
            for idx = 1:numel(names);
                plotPose(poses(idx,:), 0.1, 1, names{idx});
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


    if(~exist('port','var') || isempty(port))
        port = 1511;
    end
    
    fprintf('\n\nStarting OptiTrack on port %d\n',port);
    fprintf('Press any key to start...\n');
    pause
    optitrack = OptiTrack(port);
    optitrack.setTransform([-0.0349 0.0 0.0583 0.7071 -0.7071 0.0 0.0]);
    optitrack.start();
    pause(2);
    
    
    fprintf('\n\nPrinting Frames\n');
    fprintf('Press any key to start...\nand afterwards any key to stop...\n');
    pause
    optitrack.addCallback(@printFrame);
    pause
    optitrack.callbacks = {};
    
    
    fprintf('\n\nPlotting Frames\n');
    fprintf('Press any key to start...\n');
    pause
    figure(666);
    optitrack.addCallback(@(n,p)plotFrame(n,p,666));
    fprintf('Press any key to stop...\n');
    pause
    optitrack.callbacks = {};
    
    
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


end

