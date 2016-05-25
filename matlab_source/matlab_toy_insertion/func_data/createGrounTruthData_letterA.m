function [T, gt] = createGrounTruthData(xyzTraj, nTraj)


    xyzTraj  = interp1(linspace(0,1,numel(xyzTraj(1,:)) ), xyzTraj', linspace(0,1,nTraj) )';

    xyzTraj  = xyzTraj.*0.75;
    
    for t=1:nTraj
       gt.Torig(:,:,t) = [ [ eye(3) ; 0 0 0]   [xyzTraj(:,t); 1]];
    end
    %[out] = homogTransfPlot(gt.Torig);

    for t=1:nTraj
       gt.Torig(1:3,1:3,t) = rpy2r([-90 0  180]*pi/180)*gt.Torig(1:3,1:3,t);
    end        
    %[out] = homogTransfPlot(gt.Torig);
    
    for t=1:nTraj
       gt.Torig(1:3,1:3,t) = rpy2r([0 0  -35]*pi/180)*gt.Torig(1:3,1:3,t);
    end        
    %[out] = homogTransfPlot(gt.Torig);
    
    
    gt.shift.xyz = 1*[0.3  0.1  -0.3]';
    gt.shift.rpy =   [30        0    -30 ]*pi/180;
    gt.oTa = rpy2tr(gt.shift.rpy) + [ zeros(4,3)   [gt.shift.xyz; 0] ];

    nTraj = numel(xyzTraj(1,:));
    for t=1:nTraj
       T(:,:,t) = gt.oTa*gt.Torig(:,:,t);
    end
    homogTransfPlot(T);


end
    
    
    
    
