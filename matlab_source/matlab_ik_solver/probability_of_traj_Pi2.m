function P = probability_of_traj_Pi2(S)

    n_reps = numel(S);

    % gjm: The theoretical expS is replaced by this heuristic here. See Pi2
    %      paper ICRA 2010.
    %
    % compute the exponentiated cost with the special trick to automatically
    % adjust the lambda scaling parameter
    maxS = max(S,[],2);
    minS = min(S,[],2);
    h = 10; % this is the scaling parameters in side of the exp() function (see README.pdf)
    expS = exp(   -h*(S - minS*ones(1,n_reps))./((maxS-minS)*ones(1,n_reps))     );
    
    % the probabilty of a trajectory
    P = expS./( sum(expS,2)*ones(1,n_reps) );
    
    %sum(P)
end