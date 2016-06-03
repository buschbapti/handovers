function [hfig] = sspng(folderName, prefix, varargin)
% [hfigs] = sst([], [])
% [hfigs] = sspng(folderName, prefix)
% The same as sss. Saves only png.


    % hfig = save_QuickAndDirty(folderName, figNumbering);
    
    hfigs = get(0, 'children');
    hfigs = sort(hfigs)';
    

    figNames = [];


    for ii = 1:length(hfigs)
       figure(hfigs(ii));
       tmpHandle = get(gcf);
       if isgiven(prefix)
            figNames = [ figNames   {[prefix '_' num2str(ii) '_'  tmpHandle.Name]}];
            %figNames = [ figNames   {[prefix '_' tmpHandle.Name]}];
       else
            figNames = [ figNames   {[num2str(ii) '_' tmpHandle.Name]}];
            %figNames = [ figNames   {[tmpHandle.Name]}];
       end
    end

    % save figures
    if isgiven(varargin)
        res = ['-r' num2str(varargin{1})];
    else
        res = ['-r300'];
    end
    
    % save figures
    optionsForFigures  = struct('jpeg', 0, 'jpegRes', res, 'ps', 0,...
                                'fig',  0, 'emf', 0, 'pdf', 0, 'png', 1);
    saveFigObj = setWhatFigures2Save( hfigs, figNames, optionsForFigures );

    
    if ~isempty(folderName)
        directory = folderName;
    else
        directory = sprintf(datestr(now, '_yyyymmdd_HHMMSS'));
    end

    
    % generate folders and save data
    mkdir(directory);
    this_path = pwd;
    cd (directory);
      fprintf('====> Saving data in: %s\n', directory);
      saveFigObj.saveFiguresNow; % save figures
    cd (this_path)
    
    
end



