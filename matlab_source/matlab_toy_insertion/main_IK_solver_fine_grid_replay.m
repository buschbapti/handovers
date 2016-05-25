
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
load(['./data/fine_' type_grid '.mat']);


nTotal = numel(solGrid);
jStart = 1;
for j=jStart:nTotal
    fprintf('j: %g\n', j);
    try
        robot.setCartesian(solGrid{j-1}.T(:,:,end), 'Dummy_viaPoint_table');
    end
    robot.setCartesian(solGrid{j}.T(:,:,end), 'handoverPosition');    
    for t=1:2:numel(solGrid{j}.q(:,1))
        robot.setJointAngles(solGrid{j}.q(t,:),1);
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





































