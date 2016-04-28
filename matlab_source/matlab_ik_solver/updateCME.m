function [theta_mean_new, theta_Cov_new] = ...
                    updateCME(theta_mean, theta_Cov, theta_mean_pert, C, type)

    dbg = 0;
    switch type
        case 'CME'
            
            nRollOut=numel(C);
            nElite = round(nRollOut*0.5);
            
            % sort as CME
            [val, ind] = sort(C, 2, 'ascend');
            elite = theta_mean_pert(ind(1:nElite),:);

            K = numel(elite(:,1));
            P = (1/K).*ones(1,K);

        case 'Pi2' % sort as Pi2
            P = probability_of_traj_Pi2(C);
            elite = theta_mean_pert;
    end
    
    d2.mu  = zeros(size(theta_mean));
    d2.cov = zeros(size(theta_Cov));
    for k = 1:numel(elite(:,1))
        d2.mu  = d2.mu  + P(k)*elite(k,:);

        eps = (elite(k,:)-theta_mean)';
        epsSquare = eps*eps';
        d2.cov = d2.cov + P(k).*epsSquare;   
    end
    theta_mean_new = d2.mu;
    theta_Cov_new  = d2.cov;

    %pause(0.5);
    

end


%     if 0
%     
%         % probability of roll-out
% 
% 
%         if dbg
%             figurew
%             plot(P, C,'bo');
%             xlabel 'Probability';
%             ylabel 'Cost';
%         end
% 
%         % compute correction
%         %deltaw = weps'*P';
% 
%         deltaw = (P*weps);
% 
%         % update mean
%         theta_mean_new = theta_mean + deltaw;
% 
%         % update covariance
%         deltawCov = zeros(size(theta_Cov));
%         for k = 1 : round(numel(P)/2)
%              m1 = weps(k,:)'*weps(k,:);
%              deltawCov = deltawCov + P(k).*m1;
%         end
%         theta_Cov_new  =  theta_Cov + deltawCov;
% 
%     end
    
    
    