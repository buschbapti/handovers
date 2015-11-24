function out = interpolate_poses(in, indexes, poses)
%
%
% INPUT
%   in.T = [4x4xnTraj] a trajectory as homog. trasnf. matrices. 
%          The rotation part will be overwritten so it can be given as
%          rubish
%
%   indexes = [ nViaPoints ] the indexes of the via points where the poses
%             are given
%
%   poses   = [4x4xnViaPoints] homog transformation matrices that specify
%           the position and orientation of the via points
    
    out = in;

    for k=1:numel(indexes)-1;
        nSteps = indexes(k+1)-indexes(k)+1;
        T(:,:,indexes(k): indexes(k+1))= interpolate_two_quatertions(poses(:,:,k), poses(:,:,k+1), nSteps);    
    end
    
    out.T(1:3,1:3,:) = T(1:3,1:3,:) ;
    
end


