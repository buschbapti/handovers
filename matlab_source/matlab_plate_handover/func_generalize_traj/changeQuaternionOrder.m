function vecOut = changeQuaternionOrder(vecIn)

    vecOut = vecIn;

    idx = [1 2 3 7 4 5 6];
    for k=1:numel(vecIn(:,1))
        vecOut(k,:) = vecIn(k,idx);
    end
    
    
end