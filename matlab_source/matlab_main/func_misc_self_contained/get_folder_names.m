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
