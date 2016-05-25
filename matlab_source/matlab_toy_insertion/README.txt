

main_IK_solver_coarse.m
1. load the coarse grid from Baptiste. It only has one ball orientation.
2. create 5 ball orientations for each point in the grid
3. compute the trajectories
4. save files 
    ./data/grid_reba.mat
    ./data/grid_relative.mat

main_IK_solver_fine_grid.m
1. load
    ./data/grid_reba.mat
    ./data/grid_relative.mat
2. and compute trajectories on a finer grid



