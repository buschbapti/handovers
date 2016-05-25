
if 0
    initialize_dropbox_path(1,0,1);
    addpath('./func_robot_vrep');
    addpath('./func_robot_vrep/func_vrep_bridge');
    disp('** set ik damping to 0.01 in vrep**')
    disp('** IK weight resolution linear = 1, angular =1 **')
    pause();
end

clear; clc; close all; dbstop if error;
type_grid = 'grid_relative';
type_grid = 'grid_reba';

robot = initialize_vrep_baxter('elbow_down');
load(['./data/' type_grid '.mat']);

x = [ 0.5    0.9];
y = [-0.5    0.40];
z = [-0.35   0.20];

grid.x = linspace(x(2), x(1), 4);
grid.y = linspace(y(2), y(1), 4);
grid.z = linspace(z(2), z(1), 4);

if 0
    for k = 1:numel(grid.z)
        for j = 1:numel(grid.y)
            for i = 1:numel(grid.x)
                T = eye(4); T(1:3,4) = [grid.x(i), grid.y(j), grid.z(k)]';
                robot.setCartesian(T, 'Dummy_target');
                pause(0.1)
            end
        end    
    end
end


% generalize
for k = 1:numel(grid.z)
    for j = 1:numel(grid.y)
        for i = 1:numel(grid.x)

            % the rotations to be found in the grid should come from the
            % closes point in the original grid from Baptiste
            Tguess = find_closest_guess([grid.x(i), grid.y(j), grid.z(k)], sol);
            
            for g=1:numel(Tguess)
                T_ = Tguess{g}.TendEffOriginal(1:3,1:3);
                T_ = [ [T_; [0 0 0]]   [grid.x(i), grid.y(j), grid.z(k) 1]' ];
                gen{g}.T = T_;
                robot.setCartesian(T_, 'Dummy_target');
                robot.setJointAngles(Tguess{g}.q(end,:))
                pause(.1)
            end
        end
    end
end


break
            
            
            
            
            Txyz = eye(4); T(1:3,4) = [grid.x(i), grid.y(j), grid.z(k)]';
            
            
            
            
            
            Thandover(1:3,4) = Thandover(1:3,4) + [0.10 0.25 -0.23]'; 
            robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('Dummy_target'), 1)
            robot.sendTargetCartesianCoordinates(Thandover(1:3,4), tr2rpy(Thandover), robot.getHandle('handoverPosition'), 1)
            robot.setJointAngles(sol{k}.q(end,:));
            pause(0.25); % put IK damping 0.01
            qnew = robot.getJointAngles();

            % check desired position is achievable
            flag.e = norm(tr2delta( robot.readEntityCoordinate('Dummy_tip'), Thandover ));

            for j=1:7
                d{j} = Dmp(sol{k}.q(:,j)', 'hoffmann');
                param.xf = qnew(j);
                qnewTraj{j} = d{j}.generalize(param);
            end
            q = sync_dmps(qnewTraj);
            [flag.limMin, flag.limMax] = robot.checkJointLimit(q, 1);

            % diagnose
            diagnose(flag.e, flag.limMin, flag.limMax);

            for t = 1:numel(q(:,1))
                robot.setJointAngles(q(t,:), 1)
                pause(0.01)
            end

            dbg = 1;
            if dbg
                figurew
                plot(sol{k}.q, 'Color', [0.8 0.8 0.8], 'LineWidth', 3)
                plot( numel(q(:,1))*ones(7,1),  qnew', sty('b', 'o', 2, 'none', 10)    );
                plot(q)        
            end

            %time = linspace(0,5, );
            write_trajectory_file('./data', [linspace(0,5, numel(q(:,1)))'  q], [], 100, 2);
            % pause



break




k=4
paramFilter.filterOrder = 4; 
paramFilter.filterFreq = 0.05;
for j=1:7
    q(:,j) = smoothf( sol{k}.q(:,j), paramFilter );
end
figurew
plot(sol{k}.q, 'Color', [0.8 0.8 0.8], 'LineWidth', 4)
plot(q)

for t = 1:numel(q(:,1))
    robot.setJointAngles(q(t,:), 1)
    pause(0.01)
end
