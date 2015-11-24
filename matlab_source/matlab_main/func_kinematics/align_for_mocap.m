function [d, hfig] = align_for_mocap(xyz, hfig)
% function Tout = align_for_mocap(xyz)
% Create a trajectory of homog. transf. matrices where the reference frame
% is aligned according to the optitrack system.
%
% Usually, the Z is tangential to the trajectory (represents the velocity vector)
% and X is pointing downwards.
%
%
%
% INPUT
%   xyz = [nTraj x 3]
%
% OUTPUT
%   Tout = homog. transf. matrix
%   hfig = handle for the figure.
%
%

    % Create an initial T without orientation
    for k=1:numel(xyz(:,1))
        T(:,:,k) = [ eye(3)  xyz(k,:)'; 0 0 0 1 ];
    end    
    
    % Make X tangential to the trajectory
    [d.T] = align_ref_with_path( T, '+z' );
    
    if ~isempty(hfig)
        tmp.hfig = hfig;
        tmp.axis_length = 0.05; 
        tmp.nMaxPlots = 20;    
        tmp.axis_length = 0.1;
        homogTransfPlot(d.T, tmp);          

    end




end




























