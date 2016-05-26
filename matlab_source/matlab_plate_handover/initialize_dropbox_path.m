function [] = initialize_dropbox_path(addAllSubfolders, optionKinematic)
% function [] = initialize_dropbox_path(addAllSubfolders, optionKinematic)
% Example:
%
%   Just add dropbox functions
%   initialize_dropbox_path() 
%
%   Add all subfolders in the directory where this function is being called
%   initialize_dropbox_path(1, 0) 
%
%   Add some extra kinematic stuff from Peter Corke toolbox
%   initialize_dropbox_path(0, 1) 
%

    % list here all possible paths to the dropbox folder
    possible_folder{1} = '/localdata/Dropbox/myMatlabFunctions';
    possible_folder{end+1} = 'c:\gjm\Dropbox\myMatlabFunctions';
    possible_folder{end+1} = '~/Dropbox/myMatlabFunctions/';
    possible_folder{end+1} = 'C:\Users\mito\Dropbox\myMatlabFunctions';
    
    for k=1:numel(possible_folder)
        if isdir(possible_folder{k})
            addpath(possible_folder{k});
            fprintf('\n\nAdded: %s \n', possible_folder{k});
            
            if optionKinematic
                addpath([possible_folder{k} '/rvctools/robot']);
                addpath([possible_folder{k} '/rvctools/common']);
                addpath([possible_folder{k} '/kinematics']);
                fprintf('Added some kinematic functions.\n');
            end
            
            
        end
    end
    fprintf('\n\n\n');
    
    %addpath(genpath('../common_PCorkeRobotics'));
    if addAllSubfolders
        addpath(genpath(pwd));
    end
    
    


end