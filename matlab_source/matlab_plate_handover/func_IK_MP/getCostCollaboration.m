function [run, currentViaPointError, viaPointCostSatisfied]  = getCostCollaboration(run, param, costDeviationFromMean)
% implements a simple squared acceleration cost function with a penalty on
% the length of the parameter vector. Additionally, a via point has to be traversed at
% certain moment of time


    n_dof  = run.n_dof;
    n_traj = numel(run.dof(1).q);   % the length of a trajectory in time steps
    
    % Better to not call this independently (one call for each DoF) because depending on the cost
    % function, it may be better to have access to all joints simultaneously.
    [costPerJoint, viaPointError] = inner_loop(run, param, n_dof, n_traj);
    
    fprintf('Cost: total %g. Only deviation %g\n', sum(sum(costPerJoint))+sum(sum(costDeviationFromMean)), sum(sum(costDeviationFromMean)));
    costPerJoint = costPerJoint + costDeviationFromMean;
    
    % update 
    for j=1:n_dof
        run.dof(j).costHistory = costPerJoint(:,j);
    end
    run.full_DoF_cost = sum(costPerJoint,2); 

    % compute the via point distance error for each via point
    dSquare  = sum(viaPointError.^2,2);
    dErrorCM = 100*sqrt(dSquare);
    
    currentViaPointError = mean(dErrorCM); % in cm
    if currentViaPointError < param.ViaPointStopError
        viaPointCostSatisfied = true;
    else
        viaPointCostSatisfied = false;
    end
        
end


function [costPerJoint, viaPointError] = inner_loop(run, param, n_dof, n_real)
%
%
% OUTPUT
%   viaPointError = [ nViaPoint x nDoF] 
%                   Each column has (Xref - X) on the specific dimension.

    % compute cost
    Qacc       = param.repsCost.accel;
    Qvel       = param.repsCost.vel;
    QviaPoint  = param.repsCost.viaPoint;
    QparamSize = param.repsCost.paramSize;
    QodomCarte = param.repsCost.odometryCartesian;
    
    for j=1:n_dof
        
        Cost  = zeros(n_real,1);    
        
        % velocity cost
        velocity_cost = -run.dof(j).qdot(1:n_real);
        Cost  = Cost + Qvel*velocity_cost.^2;

        % punish longer trajectories
        odometry = cumsum(abs(run.dof(j).qdot));
        Cost = Cost + QodomCarte*odometry;
        
        % acceleration cost
        acceleration_cost = -run.dof(j).qddot(1:n_real);
        Cost  = Cost + Qacc*acceleration_cost.^2;

        % punish size of changes in parameter
        parameter_size_cost = sum(  run.dof(j).theta_eps.^2  );
        parameter_size_cost = parameter_size_cost*ones(n_real,1);
        Cost  = Cost + QparamSize*parameter_size_cost;   

        idx = find(~isnan( param.mp.viaPoint.q(:,j)   ))  ;

        if ~isempty(idx)
            viaPoint = param.mp.viaPoint.q(idx,j);
            viaPointError(:,j) = run.dof(j).q(idx)-viaPoint;
            Cost(idx) = Cost(idx) + QviaPoint*viaPointError(:,j).^2;
        end
        
        costPerJoint(:,j) = Cost;        
        
    end % for each dof

        
end






























