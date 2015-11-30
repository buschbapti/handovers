function T = filter_T(T, plotFlag)

    paramGeneral.filterOrder = 3;
    paramGeneral.filterFreq = 0.07;
    
    xyz = squeeze(  T(1:3,4,:) )' ;
    xyzS = smooth_by_filtering( xyz, paramGeneral);
    
    for k=1: numel(T(1,1,:))
        
        T(1:3,4,k) = xyzS(k,:)';
        
    end

    
    
end