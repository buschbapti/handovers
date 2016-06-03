function covStompF = covarianceStomp( nTheta, nExtra )
% function covStompF = covarianceStomp( nTheta )
% function covStompF = covarianceStomp( nTheta, nExtra )
%
% INPUT
%   nTheta: the number of parameters to be perturbed
%
%   nExtra: if it does not exist, compute the original STOMP covariance
%
%           [ hstart   hgoal  ] is a vector with two scalars. The first one
%           indicates how much perturbation you want for the weights at the
%           start. Similar for hgoal. This is a heuristic value that you
%           should set by observing the plots of the original and resulting
%           covariance. To plot, turn the flag "debugPlot" to true.
%           The magnitude of this value depends on nTheta.    
%
%
%
    debugPlot = 0;

    % Kernels from Fornberg1998 paper
    ker = [1 -2 1];
    % ker = [-1/12  4/3  -5/2  4/3  -1/12];
    % ker = [1/90 -3/20   3/2 -49/18   3/2  -3/20   1/90 ];
    
    if ~exist('nExtra', 'var') % original computation that gives zero variance 
                               % at the start and goal positions (STOMP)
        
        A = createMatrixGivenKernel(ker, nTheta);
        covStompF = inv( (A')*A  );    
        if debugPlot
            figurew('stompCov')
            plot(covStompF);
        end
        
    else % Create a covariance that has noise at the beginning and/or at the end
         
        nThetaF = nTheta + ( nExtra(1)+nExtra(2) )-1 ;
        A = createMatrixGivenKernel(ker, nThetaF);
        covStomp = inv( (A')*A  );

        nF = size(covStomp,1);
        ir = nExtra(1):nF-nExtra(2);

        if nExtra(1) == 0
            ir = ir+1;
        end            
        covStompF = covStomp( ir, ir ); 
        
        if debugPlot
            figurew('stompCov')
            plot(covStomp, sty('r', [], 4));
            plot(covStompF, 'b');
        end               
    end        
    
end



function A = createMatrixGivenKernel(A_kernel, nSize)

    
    n_kernel = numel(A_kernel);
    
    nSize = nSize + (n_kernel-1);

    A=1;
    clc
    clear tmp_start tmp_full tmp2
    
    
    
    nSizeA   = nSize+(n_kernel-1);
    
    tmp = [A_kernel   zeros(1,nSizeA)];
    
    tmp2=[];
    for k=1:nSizeA
        tmp0 = [zeros(1,k-1) tmp(1:end-k+1)];
        tmp2 = [tmp2; tmp0];
    end
    
    tmp3 = tmp2(1:nSize, n_kernel:end);
    
    % search for kernel as a column from the end of tmp3
    query = [zeros(1,nSize-n_kernel) A_kernel];

    nTmp3 = numel(tmp3(1,:) );
    
    k = nTmp3;
    while sum(abs(tmp3(:,k)'-query))~=0
        k=k-1;
    end

    tmp4 = tmp3(:, 1:k);    
   
    A = tmp4;
end

























