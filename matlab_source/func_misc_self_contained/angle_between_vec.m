function [d] = angle_between_vec( Pinit , Pgoal )
%
% Pinit ------------> Pgoal
% vector starts at P1 and goes to P2

% [d] = angleDistance(x1, x2)
% Description:
% This function returns the minimum absolute distance between two angles, independent of them being wrapped or unwrapped.
% It cannot be used for feedback control since only absolute values are returned.
% It is useful to calculate nearest neighbors in RRT.
% It uses the inner product method, which may not be efficient, but at least I think it is the safest if
% you forget to wrap the angles properly.
% May have some advantages because there is no need to wrap and the result is already unwrapped between [0 pi]
% 
% INPUT
% x1  is (M x N) of angles (rad) wrapped or unwrapped
% x2  is (P x N) of angles (rad) wrapped or unwrapped
%
% OUTPUT
% out is (1 x N) of distances (rad) wrapped between [0 pi]
% weird return the 

% get cartesian coordinates
P1x = Pinit(1);
P1y = Pinit(2);

P2x = Pgoal(1);
P2y = Pgoal(2);

% inner product between x1 and x2 is Px + Py
Px = P1x.*P2x;
Py = P1y.*P2y;


Pinit_mag = sqrt( sum(Pinit.^2) );
Pgoal_mag = sqrt( sum(Pgoal.^2) );


d = acos(   (Px + Py)/(Pinit_mag*Pgoal_mag)  ); % internal product

% have to normalize


% check if Px + Py is within the range [-1 1] for acos calculation resulting in a real angle
if isreal(d) == 0
    % error message: some values are imaginary
    disp('angleDistance: theta contains imaginary part. Inner product out of [-1 1] range.');
    disp('check variable indx to get indexes of the inner product that are bad behaved.');
    disp('if those inner products are approx 1, approximation is possible')
    badInnerProd = abs(Px+Py) >1;
    indx = find(badInnerProd);
    keyboard
end


end
