function [T, gt] = load_data3D_letterA(nTraj)

    % get the matrices from a and b letters
    load('data2.mat');

    xtraj = (data2.Gn*data2.wx)';
    ytraj = (data2.Gn*data2.wy)';
    ztraj = 0.5*sin( 2*pi*0.05*linspace(0,2*pi, numel(xtraj))  );
    

    ztraj = ztraj;
    xyzTrajOrig = [xtraj; ytraj; ztraj];

    % create a ground truth here create oTa
    [T, gt] = createGrounTruthData_letterA(xyzTrajOrig, nTraj);


end