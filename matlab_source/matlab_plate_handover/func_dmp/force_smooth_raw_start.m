
function [in] = force_smooth_raw_start(in, nExtra)

    dbg=0;

    % adding some initial states at the beginning which are just
    % repetitions
    in.p = [repmat( in.p(:,1) , [1 nExtra] )  in.p ];
    
    for k=1:nExtra
        in.T = cat(3, in.T(:,:,1), in.T);
    end

    if dbg
        figurew('test')
        plot(diff(in.p(1,:)), 'bo-')
        plot(diff(in.p(2,:)), 'ro-')
        plot(diff(in.p(3,:)), 'ko-')
    end
   
    param.filterOrder = 3; 
    param.filterFreq = 0.1;        
    for j = 1:3
        pSmooth(j,:) = smooth_by_filtering( in.p(j,:)' , param, 0)';
    end
    in.p = pSmooth;

    if dbg
        plot(diff(in.p(1,:)), 'm.-')
        plot(diff(in.p(2,:)), 'm.-')
        plot(diff(in.p(3,:)), 'm.-')
    end
    
    
    % Update in.T with smoothed xyz coordinates
    nTraj = numel( in.T(1,1,:) );
    for k=1:nTraj
        in.T(1:3,4,k) = in.p(:,k);
    end

end