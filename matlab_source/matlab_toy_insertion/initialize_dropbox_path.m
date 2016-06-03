function []   = initialize_dropbox_path(addFuncSubfolders, addAllSubfolders, optionKinematic)
% function [] = initialize_dropbox_path(addFuncSubfolders, addAllSubfolders, optionKinematic)
% Example:
%
%   Just add dropbox functions
%   initialize_dropbox_path() 
%
%   Add func subfolders(1, 0, 0) 
%
%   Add some extra kinematic stuff from Peter Corke toolbox
%   initialize_dropbox_path(0, 0, 1) 
%
%   Add all subfolders in the directory where this function is being called
%   initialize_dropbox_path(0, 1, 0) %


    if nargin == 2
        fprintf('\n');
        fprintf('*********\n');
        warning('This function changed!!! Update your code!!')
        fprintf('This function was updated and now requires 3 inputs\n');
        fprintf('[addFuncSubfolders, addAllSubfolders, optionKinematic ]\n');
        fprintf('You should not use the flag addAllSubfolders as it may add\n');
        fprintf('undesired subfolders.\n');
        fprintf('We assume all folders to be added have the string "func_"\n');
        fprintf('as part of the name\n');
        fprintf('\n');
        keyboard
    end

    % list here all possible paths to the dropbox folder
    possible_folder{1}     = '/localdata/Dropbox/myMatlabFunctions';
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
                fprintf('Added some kinematic functions.\n\n\n');
            end
            
            
        end
    end
    fprintf('\n\n\n');
    
    %addpath(genpath('../common_PCorkeRobotics'));
    if addAllSubfolders
        addpath(genpath(pwd));
    end
    
    if addFuncSubfolders

        file_names = get_folder_names('./');
        
        for k = 1:numel(file_names)
            if findstr(file_names{k},'func_')
                addpath([pwd '/' file_names{k}]);
            end
        end
        
    end
    
    
end




function [file_names] = get_folder_names(main_folder)

    files = dir([main_folder]);    
    
    file_names = [];
     
    %clear file_names
    cout = 1;
    for k = 1:numel(files)
        if strcmp(   files(k).name, '.') || strcmp(   files(k).name, '..')
        else
            if files(k).isdir
                file_names{cout } = files(k).name;
                cout = cout+1;
            end
        end
    end

    file_names =  file_names';
    
end

















