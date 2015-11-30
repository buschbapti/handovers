function [To1, po1] = homogTransfRotate( newRPY, T, param )
% [To, po] = homogTransfRotate( newxyz, T, param )
% Rotate a trajectory, path, or curve.
%
% INPUT
%
%   T [4,4, n_traj], is a sequence of n_traj homog. transf. matrices
%   representing the coordinates of a path of interest.
%
%   newRPY [3x1] is the new roll(x), pitch(y), yawl(z) angles to which to rotate the data.
%          Note that the convention for RPW follows Peter Corke's convention where RPY represents
%          the order of the multiplication of matrices, but in terms of the physical rotations the
%          order is, in fact, YPR.
%
%          LaValle's book (http://planning.cs.uiuc.edu/node102.html) actually uses the physical
%          order of rotations as a convetion. Therefore when you specify the RPY angles the matrices
%          are multiplied as YPR. This is easier to specify, but as I want to use some of the Peter
%          Corke's code, I will use his convetion.
%
%          The way to specify the rotations in Peter Corke convetion is to think backwards, that is
%          from the yawl, pitch, and roll order as physical rotations, and then reverse the order
%          when calling the function as [roll pitch yawl].
%
%   param.center_of_rotation [3x1]: this is an optional value that contains the xyz
%                                   coordinates of the point at which the data has to rotate
%                                   around. This is the Center of Rotation. If you do not provide
%                                   it, then the first element of the data will be assumed as the CR.
%
%   param.plot_flag: a flag to plot the data
%
%   param.rotation_seq. Ex: 'rpy' for roll, pitch, yawl sequence. Other examples, 'pyr', 'ypr', etc.
%                       If the field does not exist, then it will be considered rpy.
%                       Note that 'rpy' means that the order at which
%                       rotations will be applied are 'ypr' (PeterCorke
%                       convention). See the code below to understand.
%
% OUTPUT
%   
%   To [4 X 4 X n_traj]: is the new transformation matrix
%   po [3 X n_traj]:     is the coordinates of the point. This information is
%                        already contained in To, but po is more convenient to plot.
%
%  Code implemented. GJM 01.04.2015


    if ~isfield(param, 'center_of_rotation')
        shiftxyz = T(1:3,4,1);
    else
        shiftxyz = param.center_of_rotation;
       % shiftxyz = T(1:3,4,1)-shiftxyz;
    end
    Horig = [1  0  0  -shiftxyz(1); 
             0  1  0  -shiftxyz(2); 
             0  0  1  -shiftxyz(3); 
             0  0  0  1         ];

    
    Hreturn = [1  0  0  shiftxyz(1); 
               0  1  0  shiftxyz(2); 
               0  0  1  shiftxyz(3); 
               0  0  0  1         ];  
    
    Hrotx_roll  = my_trotx(newRPY(1));
    Hroty_pitch = my_troty(newRPY(2));
    Hrotz_yawl  = my_trotz(newRPY(3));
    
    if isfield(param, 'rotation_seq')
        switch param.rotation_seq
            case 'rpy'
                % PeterCorke toolbox convetion
                H_rotations = Hrotx_roll*Hroty_pitch*Hrotz_yawl;                
                % Lavalle convetion (what I used to use)
                % H_rotations = Hrotz_yawl*Hroty_pitch*Hrotx_roll;
            case 'pyr'
                H_rotations = Hroty_pitch*Hrotz_yawl*Hrotx_roll;
            case 'pry'
                H_rotations = Hroty_pitch*Hrotx_roll*Hrotz_yawl;
            case 'ryp'
                H_rotations = Hrotx_roll*Hrotz_yawl*Hroty_pitch;
            case 'ypr'
                H_rotations = Hrotz_yawl*Hroty_pitch*Hrotx_roll;
            otherwise
                error('Please implement missing cases.');
        end
    else
        % if the field does not exist, the assume roll, pitch, yawl seq.
        H_rotations = Hrotx_roll*Hroty_pitch*Hrotz_yawl; 
    end

    N = size(T,3);
    v = [0 0 0 1]';
    for k = 1:N
        % Apply in sequence:
        % 1. translating to origin (or do nothing)
        % 2. translate to desired position
        S1 = Horig*T(:,:,k);
        S2 = H_rotations*S1;
        To1(:,:,k) = Hreturn*S2;
        % get point coordinates just to plot
        tmp     = To1(:,:,k)*v; % homog. transf. matrix X vector p
        po1(:,k) = tmp(1:3); % get only the a.p coordinates
       
        tmp     = T(:,:,k)*v; % homog. transf. matrix X vector p
        p(:,k) = tmp(1:3); % get only the a.p coordinates  
    end
    
   
    
    if param.plot_flag
        figurew('result_of_rotation'); axis 'equal';
        plot3(p(1,:), p(2,:),  p(3,:), 'bo-');
        plot3(po1(1,:), po1(2,:), po1(3,:), 'ro-');
        
        if ~isfield(param, 'center_of_rotation')
            plot3(p(1,1), p(2,1),  p(3,1), sty('b', 'o', [],[], 5));          
        else
            plot3(param.center_of_rotation(1), param.center_of_rotation(2),  param.center_of_rotation(3), sty('b', 'o', [],[],10)); 
        end
    end
  


end




