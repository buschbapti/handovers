function [To] = align_ref_with_path( T, tangentAxis )
% Receive a sequence of homog. transf. matrices.
% Note that the rotation part is ignored.
% Return a new sequence of homog. transf. matrices where the reference frames are now aligned with
% the path. This is a heuristic. It is not the unique solution.
%
% You should definitely plot the solution with
%    homogTransfPlot( To, param_plot ); 
% to see the result.
%

    To=[];

    nT = size(T,3);
    Ones = eye(3);
    param_rotate.rotation_seq = 'ryp';
    param_rotate.plot_flag = 0;
    
    for k=1:nT - 1
    
        % First rotation
        % ...............................................
        % get the segment between two consecutive time steps.
        diffVec = T(1:3,4,k+1)-T(1:3,4,k);
        % compute only the yaw angle (looking from the Z axis).
        yaw(k)  =  angle_between_vec( [1 0], [diffVec(1) diffVec(2)] );
        if diffVec(2) < 0 
            % this is because the solution is computed with acos so I have to project the angle into the
            % 3rd and 4th quadrant, when needed.       
            yaw(k) = -yaw(k); 
        end

        % ..................................................
        % project the segment on the xz plane such that I can compute the pitch angle
        param_rotate.center_of_rotation = T(1:3,4,k);
        T_seg(:,:,1)   = [Ones   T(1:3,4,k);   0 0 0 1];
        T_seg(:,:,2)   = [Ones   T(1:3,4,k+1); 0 0 0 1];
        [c.T, c.p]     = homogTransfRotate( [0, 0, -yaw(k)], T_seg , param_rotate );

        % plot the segment now projected into the zx plane
    %     try delete(hcp); end
    %     hcp = [];
    %     hcp = [hcp plot3(c.p(1,:), c.p(2,:), c.p(3,:), sty('k', [], 2, [],10))];
    %     hcp = [hcp plot3(c.p(1,1), c.p(2,1), c.p(3,1), sty('k', 'o', [],[],10))];

        % Second rotation
        % ...............................................
        % compute the second angle
        diffVec   = c.p(:,2)-c.p(:,1);    
        pitch(k)  = angle_between_vec( [1 0] ,  [  diffVec(1) diffVec(3)   ] );
        if diffVec(3) < 0
            pitch(k) = -pitch(k);
        end
        r2d( pitch(k) )    ;

        if exist('tangentAxis', 'var')
            switch tangentAxis
                case '+z'
                    pitchA = pi/2;
                case '-z'
                    pitchA = -pi/2;
            end
        else
            pitchA = 0;
        end


        [T_tmp2] = homogTransfRotate( [0 , -pitch(k) + pitchA ,  yaw(k)  ], T_seg(:,:,1)  , param_rotate );

        b2.T(:,:,k) = T_tmp2;

    end
    
    To = b2.T;

end





