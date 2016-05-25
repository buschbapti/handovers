function q = sync_dmps(qnewTraj)

    for j=1:numel(qnewTraj)
        ind(j) = numel(qnewTraj{j});
    end
    
    indMax = max(ind);
    for j=1:numel(qnewTraj)
        if numel(qnewTraj{j})~= indMax
            q(:,j) = [qnewTraj{j}; qnewTraj{j}(end).*ones(indMax-numel(qnewTraj{j}-1),1)];
        else
            q(:,j) = [qnewTraj{j}];
        end
    end 
end