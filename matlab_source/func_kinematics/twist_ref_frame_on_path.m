% The next function is useful
function To = twist_ref_frame_on_path( T, twistAngle )

    nT = size(T,3);
    
    rpy = tr2rpy(T);
    
    A = [eye(3); [0 0 0]];
    
    param.plot_flag=0;
    param.rotation_seq ='ypr';
    for k=1:nT
        
        param.center_of_rotation = T(1:3,4,k); % rotate around itself;
        TA = [A [T(1:3,4,k); 1]];
        
        newrpy = rpy(k,:);
        newrpy(1) = newrpy(1) + twistAngle(k);
        To(:,:,k) = homogTransfRotate( newrpy, TA, param );
       
    end
    
    error('This function is not working! You should perhaps try to use quaternions here!');
    

end