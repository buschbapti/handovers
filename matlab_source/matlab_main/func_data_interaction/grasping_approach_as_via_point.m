%function [] = grasping_approach_as_via_point(demo.part{1}.newVP, [0 0 -0.05], 10)
function [T_appr, T_grasp] = grasping_approach_as_via_point(vp, xyzShift, nSteps)
% Create an approach trajectory of nTraj points based on a xyzShift

    T_grasp.T = vp.T;
    
    if nSteps == 0
        T_appr.T  = vp.T;
    else
        T_appr.T = move_XYZ_on_intrinsic_frame(vp.T, xyzShift);
    end
    T_appr.nSteps   = nSteps; % number of extra steps before achieving the tool
    T_appr.xyzShift = xyzShift;    
    T_appr.idx = vp.idx;
end