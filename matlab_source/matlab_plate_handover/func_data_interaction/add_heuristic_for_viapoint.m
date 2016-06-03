function [outT, outp] = add_heuristic_for_viapoint(viaPointIndex, viaPointT, viaPointp)
% This is particular to each problem.

    iapproach = 1:round(numel(viaPointIndex)/2);
    iretract  = iapproach(end)+1:numel(viaPointIndex);

    yapproach = linspace(0.2,0,numel(iapproach));
    for k=1:numel(iapproach)
        outT(:,:,k) = viaPointT;
        outT(2,4,k) = outT(2,4,k)+yapproach(k);
        outp(:,k)   = viaPointp;
        outp(2,k)   = outp(2,k)+yapproach(k);
    end
    
    yretract = linspace(0,0.2,numel(iretract));
    nj = numel(outp(1,:));
    for k=1:numel(iretract)
        outT(:,:,nj+k) = viaPointT;
        outT(2,4,nj+k) = outT(2,4,nj+k)+yretract(k);
        outp(:,nj+k)   = viaPointp;
        outp(2,nj+k)   = outp(2,nj+k)+yretract(k);
        
        %outT(1,4,nj+k) = outT(1,4,nj+k)+0.1;
        
    end    
    
end