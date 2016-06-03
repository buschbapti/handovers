
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
%type_grid = 'grid_reba';

robot = initialize_vrep_baxter('elbow_down');
load(['./data/' type_grid '.mat']);

x = [ 0.5    0.9];
y = [-0.5    0.40];
z = [-0.35   0.20];

grid.x = linspace(x(2), x(1), 4);
grid.y = linspace(y(2), y(1), 4);
grid.z = linspace(z(2), z(1), 4);


try
    load(['./data/solGrid2.mat']);
catch
    solGrid = [];
end

flattenxyz = [];
for k = 1:numel(grid.z)
    for j = 1:numel(grid.y)
        for i = 1:numel(grid.x)
            flattenxyz = [flattenxyz; grid.x(i) grid.y(j) grid.z(k)];
        end
    end
end

nTotal = numel(flattenxyz(:,1));
if ~isempty(solGrid) % compute from where to restart
    
    % get how many states
    gridStateTMP = find_closest_guess([0 0 0], sol);
    nGridState = numel(gridStateTMP);
    
    % check how many computations so far
    nComputed = numel(solGrid);
    
    % check the last grid point that was computed fully (at all nGridState)
    jStart = round(nComputed/nGridState);
    
    % checking 
    if sum(solGrid{jStart*nGridState}.TendEffOriginal(1:3,4)-flattenxyz(jStart,:)')==0
        if jStart ~= nComputed/nGridState % 
            if sum(solGrid{j*nGridState+1}.TendEffOriginal(1:3,4)-flattenxyz(j+1,:)')==0
                jStart=jStart;
            else
                error('check where to restart');
            end
        end
    else
        error('check where to restart');
    end
    ctr=jStart*nGridState;
else
    jStart=1;  ctr=1;
end



for j=jStart:nTotal  
      
    % the rotations to be found in the grid should come from the
    % closes point in the original grid from Baptiste
    gridState = find_closest_guess(flattenxyz(j,:), sol);

    for g = 1:numel(gridState) % go over the 5 possible rotations
        fprintf('angle: %g, j: %g/%g\n', g, j, nTotal );
        T_ = gridState{g}.TendEffOriginal(1:3,1:3);
        T_ = [ [T_; [0 0 0]]   [flattenxyz(j,:) 1]' ];
        robot.setCartesian(T_, 'Dummy_target');
        robot.setJointAngles(gridState{g}.q(end,:));

        close all;
        solGrid{ctr} = optim_wrap(robot, T_, []);
        ctr = ctr+1;
        save(['./data/fine_' type_grid '.mat'], 'solGrid');
    end    
end


break


k=1; j=1; i=1; ctr=1; reloaded =0;
% generalize
while k <= numel(grid.z)
    j = 1;
    while j <= numel(grid.y)
        i = 1;
        while i <= numel(grid.x)
            
            if ~isempty(solGrid) && reloaded ==0
                ctr = numel(solGrid);
                i   = solGrid{ctr}.idx(1); j = solGrid{ctr}.idx(2); k = solGrid{ctr}.idx(3);
                reloaded = 1;
            end            
            fprintf('** x, y, z: %g/%g   %g/%g   %g/%g\n', i, numel(grid.x), j, numel(grid.y), k, numel(grid.z) );
            
            % the rotations to be found in the grid should come from the
            % closes point in the original grid from Baptiste
            gridState = find_closest_guess([grid.x(i), grid.y(j), grid.z(k)], sol);
            
            for g = 1:1%numel(gridState) % go over the 5 possible rotations
                fprintf('angle: %g, x: %g/%g, y: %g/%g, z: %g/%g\n', g, i, numel(grid.x), j, numel(grid.y), k, numel(grid.z) );
                T_ = gridState{g}.TendEffOriginal(1:3,1:3);
                T_ = [ [T_; [0 0 0]]   [grid.x(i), grid.y(j), grid.z(k) 1]' ];
                robot.setCartesian(T_, 'Dummy_target');
                robot.setJointAngles(gridState{g}.q(end,:));
                
                close all;
                solGrid{ctr} = optim_wrap(robot, T_, []);
                solGrid{ctr}.idx = [i j k];
                ctr = ctr+1;
                save(['./data/solGrid.mat'], 'solGrid');
            end       
            i=i+1;
        end
        j=j+1;
    end
    k=k+1;
end





































