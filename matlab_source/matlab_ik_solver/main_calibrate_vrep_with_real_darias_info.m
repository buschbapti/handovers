    

clear; clc; close all; dbstop if error;
initialize_dropbox_path(1,0,1);
addpath('./func_robot_vrep/func_vrep_bridge');
addpath('./func_robot_vrep/func_vrep_bridge');


% Be sure to turn IK off in vrep, otherwise the tip will keep tracking the
% target!!!
% Also, tur off the option in vrep "reset scene to initial state"

dar = initialize_vrep_darias();

% at rest posture. Values measured from real Darias
jointDariasReal = [ -3.0392396e-01   -8.8781035e-01   2.0017278e+00  -1.3994223e+00   1.9908096e+00  4.8419070e-01   9.9141830e-01 ];
xyzquatEndEffDariasReal = [6.5798853e-01  -3.7834952e-01  -2.4456733e-01   2.2838357e-01  -2.3477034e-02   8.9690873e-01   3.7794775e-01];

dar.setJointAngles(jointDariasReal);


TendEff = quaternion2homogTransfMatrix(xyzquatEndEffDariasReal(4:end));
TendEff(1:3,4) = xyzquatEndEffDariasReal(1:3)';
dar.sendTargetCartesianCoordinates(TendEff(1:3,4), tr2rpy(TendEff(:,:)), dar.getHandle('Dummy_tip'), 1)

% note that at this stage, the dummy position at the hand and the joints of
% the arm are not linked because IK is off!!!

% Move the target dummy a little inside the hand 
T = eye(4);
T(1:3,4) = [0.0 -0.025 0]';
Tnew = TendEff*T;

% Rotate the dummy a little in Y axis
T = rpy2tr(d2r([0  -20  0]));
Tnew = Tnew*T;

h = cartesian_plot('dd');
homogTransfPlot(TendEff, struct('hfig', h))
homogTransfPlot(Tnew, struct('hfig', h))
dar.sendTargetCartesianCoordinates(Tnew(1:3,4), tr2rpy(Tnew(:,:)), dar.getHandle('Dummy_tip'), 1)


sqrt(sum((TendEff(1:3,4)'-Tnew(1:3,4)').^2))




