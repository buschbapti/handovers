function s = scaleCovPi2( i, nUpdate, minScale)

    a = -1/(nUpdate);
    s = a*i+1;
    
    s = max(s,minScale);

end


