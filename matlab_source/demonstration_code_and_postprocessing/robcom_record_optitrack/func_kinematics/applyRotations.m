function [T] = applyRotations( T,  incrementalRPY )
% [T] = applyRotations( originalRPY, incrementalRPY )

    param.center_of_rotation = T(1:3,4);     param.plot_flag = 0;   
    
    
    originalRPY = 0*tr2rpy(T);
    
    if 1 % apply one by one to debug better        
        T = homogTransfRotate( [originalRPY(1)+incrementalRPY(1), 0, 0], T, param );
        T = homogTransfRotate( [0, originalRPY(2)+incrementalRPY(2), 0], T, param );
        T = homogTransfRotate( [0, 0, originalRPY(3)+incrementalRPY(3)], T, param );       
    else
        T = homogTransfRotate( [originalRPY(1)+incrementalRPY(1),...
                                originalRPY(2)+incrementalRPY(2),...
                                originalRPY(3)+incrementalRPY(3)], T, param );
    end
      
end