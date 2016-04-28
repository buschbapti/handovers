
function [traj, hfig, hfig2] = plot_workspace_trajectories(reloadSol, nTraj, nSmooth)

    traj=[];
    hfig=[];
    [xyz, cz]= get_table_coordinates();
    
    if 0

                    global tableHeight;

                    % Table coordinates
                    [xyz, cz]= get_table_coordinates();
                    hfig=[];

                    if ~reloadSol

                        % Create the figure
                        hfig = figurew('cart_traj'); grid off; 
                        set_fig_position([0.667 0.0704 0.198 0.795]);

                    % Generate the robot trajectory
                    % =======================================
                        subplot(2,1,2); hold on; grid on; %axis 'equal';
                        xlabel 'x (m)'; ylabel 'y (m)';
                        title 'first draw the top view of the robot trajectory';
                        lines = plot(  xyz(:,1), xyz(:,2), sty_nl('k',[],2,'-',[]) );
                        xlim([-0.1  2.75]);  ylim([-2   2]); axis 'equal';

                        [x y] = recordTrajectory();    
                        title ' ';

                        subplot(2,1,1);  hold on; grid on; %axis 'equal';
                        ylabel 'z (m)'
                        xlabel 'x (m)'
                        title 'second draw the side view of the robot trajectory';
                        lines = plot(  xyz(:,1), xyz(:,3), sty_nl('k',[],2,'-',[]) );
                        ylim([-0.6  0.15]);  axis 'equal';

                        [ytmp, z, ~, dz, ~, ddz] = recordTrajectory();

                        % pick the via point
                        % ======================        
                        figure(hfig);
                        subplot(2,1,1);        
                        title('GINPUT active: pick the via point');
                        drawnow;
                        pause(0.1);

                        vp = ginput(1);        

                        % interpolate the XZ trajectory to fit the XY trajectory
                        ytmp_int = interp1(linspace(0,1, numel(z)), ytmp,  linspace(0,1, numel(x)) )' ;
                        z_int = interp1(linspace(0,1, numel(z)), z,  linspace(0,1, numel(x)) )' ;

                        % find the index of the via point
                        [va, idx] = min((vp(1)-ytmp_int(:,1)).^2);
                        plot(ytmp_int(idx), z_int(idx), sty('r', 'o', 1, [], 10 )); 

                        z= z_int;        
                        robotTraj = [x y z];

                        to = linspace(0,1, numel(robotTraj(:,1)));
                        robotTraj = interp1(to, robotTraj, linspace(0,1,nTraj));
                        for j=1:3
                            robotTraj(:,j) = smooth(robotTraj(:,j), nSmooth);
                        end

                        % resample for via point
                        % ====================
                        resampleRate = nTraj/numel(z_int);
                        idx = round(resampleRate*idx); 
                        robotViaPoint = [robotTraj(idx,1), robotTraj(idx,2), robotTraj(idx,3)];


                    % Generate the human trajectory
                    % =======================================    
                        figure(hfig);
                        subplot(2,1,1);
                        title ' ';

                        subplot(2,1,2);
                        title 'third draw the top view of the human trajectory';
                        [x y] = recordTrajectory();
                        title ' ';

                        subplot(2,1,1); 
                        title 'fourth: draw the side view of the human trajectory';
                        [~, z, ~, dz, ~, ddz] = recordTrajectory();
                        zint = interp1(linspace(0,1, numel(z)), z,  linspace(0,1, numel(x)) )' ;
                        z= zint;

                        humanTraj = [x y z];      

                    % resample solutions and smooth them
                    % =======================================
                        to = linspace(0,1, numel(humanTraj(:,1)));
                        humanTraj = interp1(to, humanTraj, linspace(0,1,nTraj));
                        for j=1:3
                            humanTraj(:,j) = smooth(humanTraj(:,j), nSmooth);
                        end        
                        traj.robot = robotTraj;
                        traj.robotViaPoint = idx;
                        traj.human= humanTraj;
                        save('human_robot_traj.mat', 'traj');

                    else
                        load('human_robot_traj.mat');
                    end
    end
    
    % Plot the final solution
    % =======================================    
    hfig2 = figurewe('initialSetting');    
    plot3(xyz(:,1), xyz(:,2), xyz(:,3), sty_nl('k',[],2,'-',[])) ;
    
    view([-1 -1 1.5]);
    xlabel 'x(m)'; ylabel 'y(m)'; zlabel 'z(m)';
    
    % plot the via point of the robot trajectory
    %idx = traj.robotViaPoint;
    %plot3(traj.robot(idx,1), traj.robot(idx,2), traj.robot(idx,3), sty('r', 'o',2,'-',10)) ;
    
    
end
























