function ik_satisfied = plot_and_assess_IK_solution(hfig, run_eval, i, minCostIK)

    figure(hfig); 
    subplot(3,1,1);
    plot(i, run_eval{1}{1}.ikSol.cost, styw('b', 'o', [], [], 10) );
    title(['IK cost: '  num2str(run_eval{1}{1}.ikSol.cost)] );

    
    % TODO: what is the criterio????
    if run_eval{1}{1}.ikSol.cost < minCostIK
        ik_satisfied  = true;
    else
        ik_satisfied  = false;
    end
    
    if i == -1
        ik_satisfied =  false;
    end
end