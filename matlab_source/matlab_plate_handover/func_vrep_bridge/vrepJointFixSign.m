function qout = vrepJointFixSign(qin)
% This function is used to correct the sign of the joints between SL and VREP conventions
% As it is just a multplication it works in both ways: Darias -> VREP and VREP -> Darias
%
%

    qout = qin;
    qout(:,2) = -qout(:,2); % fix sign for joint 2
    qout(:,6) = -qout(:,6); % fix sign for joint 6

end