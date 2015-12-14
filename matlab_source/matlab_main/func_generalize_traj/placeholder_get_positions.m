function [rosPositions, viaPoint, reba] = placeholder_get_positions(placeHolderParam)


    % Take original poses from Baptiste as initial starting points
    posesFromROS(1,:) = [        
        0.5	-0	-0.418522915653	0.875912087128	0.482409229925	-0.00383223594505	-0.00668314856413	
    ];

    posesFromROS(2,:) = getREBAPose(20); % select one of the poses
    posesFromROS(2,1) = posesFromROS(2,1)+0.15;
    posesFromROS(2,2) = 0;
    posesFromROS = changeQuaternionOrder(posesFromROS);

    viaPoint.T = fromQuaternionToHomog( posesFromROS(1,:));
    viaPoint.T = viaPoint.T*my_trotz(d2r(-65));
    [viaPointVec] = shake(viaPoint.T, placeHolderParam.viaPoint, placeHolderParam.deterministic);

    reba.T = fromQuaternionToHomog( posesFromROS(2,:));
    [rebaVec] = shake(reba.T , placeHolderParam.handOver, placeHolderParam.deterministic);

    rosPositions = [viaPointVec; rebaVec];

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