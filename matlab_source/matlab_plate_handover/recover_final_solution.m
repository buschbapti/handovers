function [finalSol sys param ]= recover_final_solution(folderName)
%
% Assumes that there is only one .mat file in the folder.
%

    % Load file
    fileNamesList = get_file_names(['./' folderName ]);
    for k=1:numel(fileNamesList)
        findstr(fileNamesList{k}, '.mat');
        break
    end
    disp('Loading file:');
    disp(['./' folderName '/' fileNamesList{k}])
    getPreviousSol = open(['./' folderName '/' fileNamesList{k}]);
    
    
    sys   = getPreviousSol.obj.sys;
    param = getPreviousSol.obj.param;
    robotOriginal = getPreviousSol.obj.initT;
    
    
    % Get trajectory back at original frame position
    % =============================================
    Gn = sys{2}.basis.Gn;
    for j = 1:sys{2}.nDoF, % for each dmp independently
        p(:,j) = Gn*sys{2}.dof(j).wMean(:,end);
    end    

    % Move the MP trajectory to the current IK solution frame
    % ===================================================
    nTraj = numel(p(:,1));
    
    % 0. Create the tranformation matrix while using the original end effector orientation
    for t =1:nTraj
        oldMP.T(:,:,t) = [eye(3,3) [p(t,1); p(t,2); p(t,3)] ;[0 0 0 1]] ;
    end
 
    runIK = roll_out_refFrame([], sys{1}, oldMP, [], param.frameRotateCenter, 0);
   
    
    % Maintain the original orientation of all points
    % =====================================================
    for t=1:nTraj
        runIK{1}{1}.T(1:3,1:3,t) = robotOriginal(1:3,1:3,t); 
    end
    finalSol = runIK{1}{1};
    
end