clear
close all
clc
dbstop if error

addpath('~/Dropbox/myMatlabFunctions/');

global darias

% ./xrdarias -rightArm -hands

try
    darias.stop();
end
try
    darias.delete();
end
recover_java_crash;


testDarias_gjm '130.83.164.51'

 










