%TROTZ Rotation about Z axis
%
% T = TROTZ(THETA) is a homogeneous transformation (4x4) representing a rotation 
% of THETA radians about the z-axis.
%
% T = TROTZ(THETA, 'deg') as above but THETA is in degrees.
%
% Notes::
% - Translational component is zero.
%
% See also ROTZ, TROTX, TROTY, TROT2.

function [T, R] = my_trotz(t)

    ct = cos(t);
    st = sin(t);
    R = [
        ct  -st  0
        st   ct  0
        0    0   1
        ];

    
    
	T =    [R [0 0 0]'; 0 0 0 1];
    
end