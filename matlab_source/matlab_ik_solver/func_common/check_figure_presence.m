
function stopFlag = check_figure_presence(hFig)

    stopFlag=0;
    try
        ishandle(hFig);
    catch
        stopFlag=1;        
        fprintf('Stopping loop\n');
        %keyboard
    end
    
    
end