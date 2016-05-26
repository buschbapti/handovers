function out = getPositionFromT(T)
% Given a trajectory of homog. transf. matrices, return the position in terms of xyz coordinates

    nT = size(T,3);
    for t=1:nT
        tmp = T(:,:,t)*[0 0 0 1]';
        out(:,t) = tmp(1:3);
    end

end