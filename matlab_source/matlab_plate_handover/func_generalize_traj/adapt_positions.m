function [posesMatlabFormat] = adapt_positions(posesMatlabFormat, adjustAngle)


    rebaT = fromQuaternionToHomog( posesMatlabFormat(2,:));
    %rebaT2  = rebaT*my_troty( d2r(adjustAngle) );
    % rebaT2  = rebaT*my_trotz( d2r(adjustAngle) );
    rebaT2  = rebaT*my_trotx( d2r(adjustAngle) );
 
    quat_temp = Quaternion( rebaT2(1:3,1:3));
    
    % use it in normal matlab order
    vec = [rebaT2(1:3,4)' quat_temp.s quat_temp.v];

    posesMatlabFormat(2,:) = vec;

end

function [vec] = shake(T, param, deterministic)

    

    stdPos = param.stdPos;
    
    if deterministic
        xyz = T(1:3,4) + [stdPos]';
    else
        varPos = stdPos.^2;
        
        for j=1:3
            range1 = -varPos(j)/2;
            range2 = +varPos(j)/2;
            eps(j,:) = range1 + (range2-range1).*rand(1,1);
        end
        xyz = T(1:3,4) + eps;
        
        % xyz = T(1:3,4) + [varPos(1)*rand(1,1); varPos(2)*randn(1,1); varPos(3)*randn(1,1)];
    end
    
    
    % Shake angles
    Tnew  = T;
    
    stdRot = param.stdRot;
    if deterministic        
        noiseX = stdRot(1);
        noiseY = stdRot(2);
        noiseZ = stdRot(3);         
    else
        for j=1:3
            range1 = -stdRot(j)/2;
            range2 = +stdRot(j)/2;
            eps(j,:) = range1 + (range2-range1).*rand(1,1);
        end        
        noiseX = eps(1);
        noiseY = eps(2);
        noiseZ = eps(3);
    end

    Tnew = Tnew*my_trotx( noiseX );
    Tnew = Tnew*my_troty( noiseY );
    Tnew = Tnew*my_trotz( noiseZ );

    quat_temp = Quaternion( Tnew(1:3,1:3));
    
    % write in the order of ROS convention
    vec = [xyz' quat_temp.v quat_temp.s];
    
    
    
end