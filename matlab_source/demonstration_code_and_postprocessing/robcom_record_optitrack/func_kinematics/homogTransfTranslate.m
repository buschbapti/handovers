function [To, po] = homogTransfTranslate( newxyz, T, param )
% [To, po] = homogTransfTranslate( newxyz, T, type, plot_flag )
% Translates a trajectory, path, or curve.
%
% INPUT
%
%   T [4,4, n_traj], is a sequence of n_traj homog. transf. matrices
%   representing the coordinates of a path of interest.
%
%   newxyz [3x1] is the new xyz coordinates of the origin of the data
%
%   param.type: 'absolute' or 'relative' indicates if newxyz is the absolute or
%                relative position of the data.
%
%   param.ref_frame [3x1]: this is an optional value that contains the xyz
%                          coordinates of the reference frame of the
%                          curve. If the field does not exist, then it will
%                          use the coordinates of the first element in T as
%                          the origin. That is, it will assume that the
%                          reference is attached to the first element of
%                          the trajectory.
%
%   param.plot_flag: [1 or 0] a flag to plot the data
%
% OUTPUT
%   
%   To [4 X 4 X n_traj]: is the new transformation matrix
%   po [3 X n_traj]:     is the coordinates of the point. This information is
%                        already contained in To, but po is more convenient to plot.




    switch param.type
        case 'absolute' % use this to translate to the origin
            
            if ~isfield(param, 'ref_frame')
            Horig = [1  0  0  -T(1,4,1); 
                     0  1  0  -T(2,4,1); 
                     0  0  1  -T(3,4,1); 
                     0  0  0  1         ];
            else
            Horig = [1  0  0  -param.ref_frame(1); 
                     0  1  0  -param.ref_frame(2); 
                     0  0  1  -param.ref_frame(3); 
                     0  0  0  1         ];
            end
        
        case 'relative' % do nothing
        Horig = [1  0  0  0; 
                 0  1  0  0; 
                 0  0  1  0; 
                 0  0  0  1         ];
    end
    
    Hnew = [1  0  0  newxyz(1); 
            0  1  0  newxyz(2); 
            0  0  1  newxyz(3); 
            0  0  0  1         ];

    N = size(T,3);
    v = [0 0 0 1]';
    for k = 1:N
        % Apply in sequence:
        % 1. translating to origin (or do nothing)
        % 2. translate to desired position
        To(:,:,k) = Horig*Hnew*T(:,:,k);

        % get point coordinates just to plot
        tmp     = To(:,:,k)*v; % homog. transf. matrix X vector p
        po(:,k) = tmp(1:3); % get only the a.p coordinates
        
        tmp     = T(:,:,k)*v; % homog. transf. matrix X vector p
        p(:,k) = tmp(1:3); % get only the a.p coordinates  
    end
    
    if param.plot_flag
        figurew('result_of_translation'); axis 'equal';
        plot3(p(1,:), p(2,:),  p(3,:), 'bo-');
        plot3(po(1,:), po(2,:), po(3,:), 'ro-');
        if ~isfield(param, 'ref_frame')
            plot3(p(1,1), p(2,1),  p(3,1), sty('b', 'o', [],[],10));          
            plot3(po(1,1), po(2,1), po(3,1), sty('r', 'o', [],[],10));
        else
            plot3(param.ref_frame(1), param.ref_frame(2),  param.ref_frame(3), sty('b', 'o', [],[],10)); 
        end
    end
  


end