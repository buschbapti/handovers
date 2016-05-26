function computer = get_monitor_layout( dualMonitorLayout )
% computer = get_monitor_layout( layout )
% 
% Return the name of the computer and the layout of the monitors.
% This function is useful if you set figures according to a pre-specified
% layout in your monitor.
% 
% INPUT
%    dualMonitorLayout: if exist, it shoudl be either horizontal or
%                       vertical.
%                       If not given, it will be set as unknown.

    [~, computerName] = system('hostname');
    tmp = cellstr(computerName);
    computer.name = tmp{1};
    
    if exist('dualMonitorLayout', 'var')
        
        if ~strcmp(dualMonitorLayout, 'vertical') && ~strcmp(dualMonitorLayout, 'horizontal')
            fprintf('\n\n\n\n');
            warning('Monitor layout not recognized. Options are "vertical" or "horizontal".');  
            warning('Layout set to "unknwon".');  
            dualMonitorLayout = 'unknown';
        end
        computer.layout = dualMonitorLayout;
    else
        computer.layout = 'unknown';
    end
    
end