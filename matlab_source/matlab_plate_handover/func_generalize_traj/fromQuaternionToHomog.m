function T = fromQuaternionToHomog( q )

    R = quaternion2homogTransfMatrix(q(1,4:7));
    T = [R(:,1:3) [q(1,1:3)'; 1]];
    
end