function [] = pause_cummulative(varargin)

    global pauseScale;

    if isempty(pauseScale)
        pauseScale=1;
    end
    
    nSteps = numel(varargin);    
    tmp = [];
    for k=1:nSteps
        tmp = [tmp; varargin{k}.data(:,1)];
    end
    totalTime = sum(tmp);
    
    if ~isempty(pauseScale)
        totalTime = totalTime*pauseScale;
    end
    
    fprintf('\n...waiting for %g (sec)\n', totalTime);   
    
    pauseType = 'normal';
    myPause(totalTime, pauseType);


end