function T1 = move_XYZ_on_intrinsic_frame(T, xyzShift)

    % Store rotations and position
    % get RPY so that I can recover it later
    savei.rpy = tr2rpy(T);
    savei.xyz = T(1:3,4);
    
    % do the shift in the desired X direction. The shift is made in a
    % reference frame aligned to the world frame (all rotations are zero)
    Tzero = [ [eye(3); [0 0 0]] [ savei.xyz + xyzShift; 1] ];
    
    % Now I have to recover the previous orientation
    param2.center_of_rotation = savei.xyz;
    param2.plot_flag = 0;
    T1 = homogTransfRotate(savei.rpy, Tzero, param2);

end