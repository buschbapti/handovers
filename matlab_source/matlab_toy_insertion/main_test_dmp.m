
function [] = main()

clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);
addpath('./func_robot_vrep');
addpath('./func_robot_vrep/func_vrep_bridge');


Tstart = eye(4);
Tend   = eye(4);
%Tend(1:3,4) = [1 1 1]';
Tend = Tend*se3(0,0,d2r(45), 1, 1, 1);

Tout = goTo(Tstart, Tend, 200);


param.hfig = cartesian_plot('f');
homogTransfPlot(Tout, param);

h.p = figurew('pos'); 
h.v = figurew('vel'); 
for j=1:3
    figure(h.p);
    subplot(3,1,j); hold on; grid on;    
    plot(  squeeze(Tout(j,4,:)   ), 'bo-'  );
    figure(h.v);
    subplot(3,1,j); hold on; grid on;
    plot( diff( squeeze(Tout(j,4,:))   ), 'bo-'  );
    d{j} = Dmp(squeeze(Tout(j,4,:))', 'hoffmann');    
end

xyz = [   squeeze(Tout(1,4,:))';  squeeze(Tout(2,4,:))'; squeeze(Tout(3,4,:))'  ] ;
xf = [2:1:10];
xf = [xf; xf; xf];

for n=1:numel(xf(1,:))
    clear xyz2
    for j=1:3
        paramdmp.xf = xyz(j,end)+xf(j,n);
        xyz2(j,:) = d{j}.generalize(paramdmp);
        figure(h.p);
        subplot(3,1,j); hold on; grid on;    
        plot(  xyz(j,:) , 'ro-'  );    
        figure(h.v); subplot(3,1,j);
        plot(  diff( xyz(j,:) )  , 'ro-');
    end
    xyz = xyz2;
end

for t =1: numel(Tout(1,1,:))
   Tnew(:,:,t) = Tout(:,:,t);
   Tnew(1:3,4,t) = xyz(:,t);
end
homogTransfPlot(Tnew, param);



end


function out = goTo(Tstart, Tend, nTraj)
% Linear interpolation of homog transf matrices
% in XYZ with quaternion interpolation for the rotations
%

    if 0
        x_ = linspace(Tstart(1,4), Tend(1,4), nTraj);
        y_ = linspace(Tstart(2,4), Tend(2,4), nTraj);
        z_ = linspace(Tstart(3,4), Tend(3,4), nTraj);
    else
        x_ = goToJointTrapez(Tstart(1,4), Tend(1,4), nTraj, []);
        y_ = goToJointTrapez(Tstart(2,4), Tend(2,4), nTraj, []);
        z_ = goToJointTrapez(Tstart(3,4), Tend(3,4), nTraj, []);
    end

    % buid the input homog. transf. matrices
    for k =1:nTraj
        in.T(:,:,k) = eye(4);
        in.T(1:3,4,k) = [x_(k) y_(k) z_(k)]';
    end

    poses(:,:,1) = Tstart;
    poses(:,:,2) = Tend;
    indexes = [1 nTraj];
    traj = interpolate_poses(in.T, indexes, poses);

    out = traj;        
end

function yf  = goToJointTrapez(xi, xf, nSteps, rates)

    if isempty(rates)
        rates = 0.25;
    end

    nStart = round(nSteps*rates(1));
    yd1 = linspace(0, 1, nStart);
    yd3 = yd1(end:-1:1);
    yd2 = linspace(1,1,nSteps-2*(nStart));

    yd = [yd1 yd2 yd3];

    y = cumsum(yd);
    y = y/max(y);

    amp = xf-xi;
    yf = (xi+y*amp)';

    dbg=0;
    if dbg
        figurew('vel profile');
        plot(yd);
        plot(y, 'r');

        figurew('final sol');
        plot(yf, sty('b', 'o', 2));
    end   

end



    
    
    
    