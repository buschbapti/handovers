function [Tout] = interpolate_two_quatertions(T1, T2, nSteps)
% [Tout] = interpolate_two_quatertions(T1, T2, nSteps)
%
% Given two T1 and T2 transformation matrices it will interpolate their quaternions with nSteps.
% The XYZ trajectory will be simply a line. Maybe it is worht implementing a different XYZ interpolation
% This function relies on the toolbox from Peter Corke.
%
%    T1: initial transf. matrix
%    T2: final   transf. matrix
%    nSteps: The number of steps between the two states
%
%

    quat1 = Quaternion(T1);
    quat2 = Quaternion(T2);
    
    if 0 % check that encoding of quaterion is correct
        hcheck = figurewe('feef');
        param_plot.axis_length = 1;
        param_plot.nMaxPlots = 10;
        param_plot.hfig = hcheck;
        homogTransfPlot(  [T1(1:3,1:3) [0 0 0]'; [0 0 0 1] ]  , param_plot );    
        quat1.plot(    struct('color', 'b', 'frame', 'F', 'view', [1 -1 -.5] )   );
    end
    
    
    % trajectory in xyz
    b = [ linspace(T1(1,end), T2(1,end), nSteps);
          linspace(T1(2,end), T2(2,end), nSteps);
          linspace(T1(3,end), T2(3,end), nSteps)];
      
    Svec = linspace(0,1,nSteps);
    for k = 1:nSteps
        q3   = quat1.interp(quat2, Svec(k));
        tmp  = quaternion2homogTransfMatrix(q3);
        Tout(:,:,k) = [tmp(:,1:3) [b(:,k); 1] ];
    end
    
     
    
end

