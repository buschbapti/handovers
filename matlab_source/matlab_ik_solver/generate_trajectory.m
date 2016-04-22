
function [Xnew, letterAr ] = generate_trajectory(theta, letterA)

    oTa = rpy2tr(theta(4:6)') + [ zeros(4,3)   [theta(1:3); 1] ];    
    letterAr = oTa*[ letterA; [ones(1,numel(letterA(1,:)) )] ];
    letterAr(4,:) = [];
    
    param.t=0;
    
    % regress DMP on transformed letter
    xdmp = dmp_regression(letterAr(1,:), param);
    xdmp.w = xdmp.w + theta(13:13+19);
   
    ydmp = dmp_regression(letterAr(2,:), param);
    ydmp.w = ydmp.w + theta(33:33+19);
    
    zdmp = dmp_regression(letterAr(3,:), param);
    zdmp.w = zdmp.w + theta(53:53+19);

    % generalize
    paramgen.xi = theta(7);
    paramgen.xf = theta(10);
    x2    =  dmp_generalize(xdmp, paramgen);

    paramgen.xi = theta(8);    
    paramgen.xf = theta(11);
    y2    =  dmp_generalize(ydmp, paramgen);

    paramgen.xi = theta(9);    
    paramgen.xf = theta(12);
    z2    =  dmp_generalize(zdmp, paramgen);    
    
    Xnew = [x2; y2; z2];
    
    
end