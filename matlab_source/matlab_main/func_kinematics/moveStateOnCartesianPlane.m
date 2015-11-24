function [x, y, z, Tnew] = moveStateOnCartesianPlane(T, plane, angle, distance )
% 
%
% INPUT
%    T: the current homog. transf. matrix that is going to be moved.
%    distance: [N x 1] is a vector with a sequence of distances 
%    
% OUTPUT
%    T(4,4,N): is the homog. transf. matrix corresponding to the sequence
%    of distances. The values of T should be then added to another matrix.

    nTraj = numel(distance);
    for k=1:nTraj
        
        switch plane
            case 'xy'
                x(k) = distance(k)*cos(angle);
                y(k) = distance(k)*sin(angle);
                z(k) = 0;
            case 'yz'
                x(k) = 0;
                y(k) = distance(k)*cos(angle);
                z(k) = distance(k)*sin(angle);
            case 'xz'
                x(k) = distance(k)*cos(angle);
                y(k) = 0;
                z(k) = distance(k)*sin(angle);
            otherwise
                error('Options are xy, yz, xz.');           
        end

        Tnew(:,:,k) = T + [ zeros(4,3) [x(k) y(k) z(k) 1]' ];

    end
end