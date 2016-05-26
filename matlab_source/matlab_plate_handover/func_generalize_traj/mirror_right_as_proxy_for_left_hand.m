function quatL_ros = mirror_right_as_proxy_for_left_hand(quatR_ros)
%  
%
% INPUT
%   quatR: quatertion for the right hand in raw ROS format.


    dbg=0;
    quatR = changeQuaternionOrder(quatR_ros);
    RT = fromQuaternionToHomog( quatR );

    % get the angles in world frame
    if dbg
        h_ = figurewe('angles');
    else
        h_ = [];
    end
    angles1 = homogTransfMatrixProjectedAngles(RT, h_ );

if 0
    % Rotate in world Z
    deltaRPY = -wrap2pi(angles1.vectorZ.aroundWorldXYZ(3))-wrap2pi(angles1.vectorZ.aroundWorldXYZ(3));
    tmp.center_of_rotation =  RT(1:3,4); 
    tmp.plot_flag = 0;
    RL1 =  homogTransfRotate( [0 0 wrap2pi(deltaRPY)], RT, tmp );

    RL1(2,4) = RL1(2,4)-0.5;
    angles2 = homogTransfMatrixProjectedAngles(RL1, h_ );
end

if 1

    % Rotate in world X
    deltaRPY = -wrap2pi(angles1.vectorY.aroundWorldXYZ(1))-wrap2pi(angles1.vectorY.aroundWorldXYZ(1));
    tmp.center_of_rotation =  RT(1:3,4); 
    tmp.plot_flag = 0;
    RL2 =  homogTransfRotate( [wrap2pi(deltaRPY) 0 0], RT, tmp );

    RL2(2,4) = RL2(2,4)-0.5;
    angles2 = homogTransfMatrixProjectedAngles(RL2, h_ );

end

    quatt = Quaternion(RL2);

    quatL_ros = [RL2(1:3,4)' [quatt.v quatt.s   ] ] ;

end

