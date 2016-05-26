
This code is used to 
1. create a grid based on some reference poses provided by Baptiste.
2. generalize the grid based on requests coming from the Reba assessment online.

The scene file in vrep changed to 
../human_baxter_vrep331.ttt
This file requires vrep version 3.3.1


main_IK_solver_coarse.m
1. load the coarse grid from Baptiste. It only has one ball orientation for each grid position.
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

main.m
1. wait for the flag from ROS
2. generate a trajectory and save it in the folder
/tmp/matlab_bridge




