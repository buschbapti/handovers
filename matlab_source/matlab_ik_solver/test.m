
clear; clc; close all; dbstop if error;
%clear classes;
initialize_dropbox_path(1,0,1);

t = 0:0.1:2*pi*0.5;

x = sin(t);

x2 = goToTrapez(x(end), 0, 40, [])';
x2 = [x x2];
figurew()
plot(x)
plot(x2, 'r')

paramFilter.filterOrder = 3;
paramFilter.filterFreq = 0.2;
x3 = smoothf(x2, paramFilter);
plot(x3, 'k')

figurew
plot(diff(x2))
plot(diff(x3), 'k')

break

goToTrapez(sol_shelf{k}.qRev(end,j), qhandoverPlate(j), 40, []);




% ThandoverPlate = ThandoverPlate*se3(0, d2r(15), 0, 0,0,0);
% compute way to reposition the arm if it is elbow up during the
% grasping

for k=1:7
    ik.sendTargetCartesianCoordinates(sol_shelf{k}.T(1:3,4, end), tr2rpy(sol_shelf{k}.T(:,:, end)), ik.getHandle('Dummy_target'), 1);
    ik.setJointAngles(sol_shelf{k}.q(end,:));


    nTraj = numel(sol_shelf{k}.q(:,1));
    sol_shelf{k}.qRev = sol_shelf{k}.q(end:-1:1,:);
    sol_shelf{k}.qRev = sol_shelf{k}.qRev(1:round(nTraj*1/2),:);

    clear qConnect
    for j=1:7
        qConnect(:,j) = goToTrapez(sol_shelf{k}.qRev(end,j), qhandoverPlate(j), 40, []);
    end

    sol_shelf{k}.qRev = [sol_shelf{k}.qRev; qConnect];


    for t = 1:round(numel(sol_shelf{k}.qRev(:,1))*(1))
        ik.setJointAngles(sol_shelf{k}.qRev(t,:));
        pause(0.05)
    end    
end
