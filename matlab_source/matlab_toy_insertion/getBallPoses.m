function q = getBallPoses(qin)

    % last joints of baxter in degrees
    min_  = -175;
    rang = 350;
    max_ = min_+rang;

    % steps to rotate in degrees
    degStep = 360/5;

    qnow = r2d(qin(end));
  
    % find minimum range
    currAngleMin = qnow;    
    while currAngleMin(end) >= min_
        currAngleMin(end+1) = currAngleMin(end)-degStep;
    end
    if currAngleMin(end) < min_
        currAngleMin(end) = [];
    end
    
    % find max_ range
    currAngleMax = qnow;    
    while currAngleMax(end) <= max_
        currAngleMax(end+1) = currAngleMax(end)+degStep;
    end
    if currAngleMax(end) > max_
        currAngleMax(end) = [];
    end
    
    currAngle = [currAngleMin(end:-1:2) currAngleMax];
    
    % check if it was successfull
    if sum(diff(currAngle)-degStep) > 0.0001
        warning('The steps between angles differ from degStep');
    end
    
    if numel(currAngle) ~= 5
        warning('There should be 5 elements');        
    end
    
    if currAngle(1) < min_
        warning('Minimum angle is out of joint limits');        
    end
    if currAngle(end)> max_
        warning('Maximum angle is out of joint limits');        
    end
    
    
    for k=1:5
        q(k,:) = [ qin(1:6)  d2r(currAngle(k)) ];
    end
    
    
    
end









