function [] = plot_MP_solution(hfig, run_eval, viaPointError, i)

    figure(hfig); 
   
    trajCost = sum(run_eval{1}{2}.full_DoF_cost);
    subplot(3,1,2);
    plot(i, trajCost, sty('r', 's', [],[], 5) );
    title(['Traj cost: '  num2str(trajCost)] );
    
    
    subplot(3,1,3);
    plot(i, viaPointError, sty('r', 'o', [],[], 5) );
    title(['Error (cm): '  num2str(viaPointError)] );
    axis 'tight';

    
    
end