function [out] = fix_acos_angle(x, y, theta)
% Receive angles wrapped [0 pi] and return [-pi pi].
% It works by assuming that the pair xy is on the geometric circle with
% radius one and centered at zero.
    
    if x>=0 && y >=0
        out = theta;
    end
    if x<0 && y >=0
        out = theta;
    end
    if x<0 && y <0
        out = -theta;
    end
    if x>=0 && y <0
        out = -theta;
    end    
end