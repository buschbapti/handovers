function sys = sys_structure(ProMP, param, useGuessFile)
% function sys = sys_structure(ProMP, param)
%
% Creates a holder for the main parameters that are optimized by REPS.
%
% sys(1): refers to IK related parameters
% sys(2): refers to MP related parameters




    % IK structure
    % =================================
    i=1;
    sys{i}.name   = 'ref_frame_IK';
    sys{i}.active = true;
    sys{i}.nBasis = 1;

    % noise variance of reference frame
    %                             x         y         theta
    wCovVec =                1.5*[  0         0       d2r(1.0)];
    for j=1:3
        sys{i}.dof(j).wMean       = [];
        sys{i}.dof(j).wMean_ = 'Initial mean policy';
        sys{i}.dof(j).wCov(:,1)   = wCovVec(j);
        sys{i}.dof(j).wCov_ = 'This is exploration noise that is added to perturb parameters.';
    end
    sys{i}.nDoF = numel(sys{i}.dof);
    
    % fill the initial mean values in sys
    sys{i}.dof(1).wMean = 0;
    sys{i}.dof(2).wMean = 0;
    sys{i}.dof(3).wMean = 0;  
    
    

    % MP structure
    % ====================================
    i=2;
    sys{i}.name     =  'trajectory_MP';
    sys{i}.active   =  true;
    sys{i}.Cov      =  ProMP.w.cov_full;
    sys{i}.basis    =  ProMP.basis;
    sys{i}.nBasis   =  ProMP.nBasis;

    for j=1:3 % dofs of interest are x  y z
        % uncorrelate dofs
        sys{i}.dof(j).dt     = ProMP.dt;
        sys{i}.dof(j).nBasis = ProMP.nBasis;
        sys{i}.dof(j).wMean  = ProMP.w.mean_full(ProMP.w.index{j});
        sys{i}.dof(j).wCov(:,:,1) = ProMP.w.cov_full(ProMP.w.index{j}, ProMP.w.index{j});
    end                                          
    sys{i}.nDoF = numel(sys{i}.dof);

    % Modify the covariance matrix of each joint to explore as in STOMP (see STOMP ICRA2011 paper)
    if param.reps.useSTOMPCovariance
        stompCov = covarianceStomp( ProMP.nBasis, param.reps.clipStompCovariance );
        for j=1:3
            sys{2}.dof(j).wCov(:,:,1) = param.reps.covarInitialScale*stompCov;
            if param.reps.noiseCovarianceDiagonalOnly
                sys{2}.dof(j).wCov(:,:,1) = diag(diag(sys{2}.dof(j).wCov(:,:,1)));
            end
        end
    end

    if useGuessFile
        [~, sysGuess ] = recover_final_solution(useGuessFile);
        for j=1:3 %
            sys{1}.dof(j).wMean  = sysGuess{1}.dof(j).wMean;
            sys{2}.dof(j).wMean  = sysGuess{2}.dof(j).wMean;
        end           
    end
    
    
    
end

















