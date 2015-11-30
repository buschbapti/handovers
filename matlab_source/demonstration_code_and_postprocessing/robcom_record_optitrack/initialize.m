function [] = initialize()

    possible_folder{1} = '/localdata/Dropbox/myMatlabFunctions';
    possible_folder{2} = 'c:\gjm\Dropbox\myMatlabFunctions';
    possible_folder{3} = '~/Dropbox/myMatlabFunctions/';
    possible_folder{4} = '~/projects/myMatlabFunctions/';
    
    counter=0;
    for k=1:4
        if isdir(possible_folder{k})
            addpath(possible_folder{k});
            addpath([possible_folder{k} '/rvctools/robot']);
            addpath([possible_folder{k} '/rvctools/common']);
            addpath([possible_folder{k} '/kinematics']);      
            fprintf('Added: %s \n', possible_folder{k});
            counter=1;
        end
    end
    
    if counter==0
        warning('No path was added!!!');
    end
    
    
    addpath(genpath('../common_PCorkeRobotics'));
    addpath(genpath(pwd));


end