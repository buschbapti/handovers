function [cost, haux] = compute_cost_deviation_mean(run, meanTraj, param, haux)

    Qdeviation = param.repsCost.deviationFromMean;

    
    % move all trajectories first to the origin
    ref  = bsxfun(@minus, meanTraj, meanTraj(:,1));
    
    curr = [run.dof(1).q  run.dof(2).q  run.dof(3).q]';
    curr = bsxfun(@minus, curr, curr(:,1));
    
    deviationFromMean = (ref-curr)';
    
    mean_deviation = mean(sqrt(sum(deviationFromMean.^2,2)));
    
    cost  = Qdeviation.*deviationFromMean.^2; 

    
    if exist('haux', 'var')
        try delete(haux.plots);  end
        haux.plots=[];
        
        figure(haux.h); 
        subplot(2,1,1); 
        title(['Mean deviation (cm)', num2str(mean_deviation*100)])
        haux.plots = [haux.plots plot(ref(1,:), ref(2,:), styw('b', 'o', 1, [], 15))];
        haux.plots = [haux.plots plot(curr(1,:), curr(2,:), sty('r', 'o', 1, [], 10))];
        
        subplot(2,1,2); 
        haux.plots = [haux.plots plot(ref(1,:), ref(3,:), styw('b', 'o', 1, [], 15))];
        haux.plots = [haux.plots plot(curr(1,:), curr(3,:), sty('r', 'o', 1, [], 10))]; 
        
        
    else
        haux=[];
    end

end  