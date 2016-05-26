function [ run ] = roll_out_trajectory(sys, run, param, noise_mult)
% function run = roll_out_trajectory(sys, run, param, noise_mult)
%
% This function does roll-outs of trajectories for motion planning.
% For the cases of letter "a" and golf, the "q"s represent Cartesian coordinates. It will return the
% XYZ coordinates in each of the q fields, for a trajectory on its original reference frame.
%
%       
% OUTPUT
%   costDeviationFromMean: returns the cost for deviating from the mean
%   trajectory. Therefore there is no need to move the location of the
%   original mean demonstration as it will be in relation to the original
%   guess.

    % make equations clean
    Gn     = sys.basis.Gn;
    Gndot  = sys.basis.Gndot;
    Gnddot = sys.basis.Gnddot;
        
    for j = 1:run(1).n_dof, % for each dmp independently

        % Select which type of covariance to use
        if param.reps.decreaseCovarianceAsPi2
            wCov = sys.dof(j).wCov(:,:,1); % use always the initial covariance.
            epsilon = mvnrnd(zeros(1, sys.nBasis), noise_mult.*wCov);
        else
            wCov = sys.dof(j).wCov(:,:,end); % use the lates covariance updated by REPS
            epsilon = mvnrnd(zeros(1, sys.nBasis), 1.*wCov);
        end


        run.dof(j).q         = Gn*sys.dof(j).wMean(:,end)    + Gn*epsilon';
        run.dof(j).qdot      = Gndot*sys.dof(j).wMean(:,end) + Gndot*epsilon';
        run.dof(j).qddot     = Gnddot*sys.dof(j).wMean(:,end)+ Gnddot*epsilon';
        run.dof(j).epsilon   = epsilon;
        run.dof(j).theta_eps = sys.dof(j).wMean(:,end)' + epsilon;
    end
        
    % compute the deformation of the trajectory in relation to the original
    % one, where both are still in the initial IK frame
    % costDeviationFromMean = compute_cost_deviation_mean(run, param);

      
end
% 
% function cost = compute_cost_deviation_mean(run, param)
% 
%     Qdeviation = param.repsCost.deviationFromMean;    
% 
%     
%     for j=1:3 % only care about XYZ directions. The orientation of each point is already
%               % given by construction.
%         deviationFromMean = (robot.initialCartesianTrajectory.p(j,:)'-run.dof(j).q);
%         cost(:,j)  = Qdeviation.*deviationFromMean.^2;         
%     end
% 
% end
















