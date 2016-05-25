function vpMid = get_via_point(T, pertxy)

    xyz = ginput(1);
    
    dist = bsxfun(@minus, squeeze(T(1:2,4,:)), xyz');
    [val, idx] = min(sqrt(sum(dist.^2)));
    
    plot(T(1,4,idx), T(2,4,idx), sty([.8 .8 .8], 'o', 1, [], 10)  );
    
    
    xy = [T(1,4,idx); T(2,4,idx)] + pertxy;
    Tmid = [[T(1:3,1:3,idx); [0 0 0]]    [xy;1;1]]  ;
    homogTransfPlot(Tmid, struct('hfig', gcf))
    
    vpMid.T   = Tmid;
    vpMid.xyz = [xy;0];
    vpMid.idx = idx;
    

end