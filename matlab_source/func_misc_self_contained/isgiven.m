function [out] = isgiven(a)
% [out] = isGiven(a)
% This is to make sintax clear. Instead of using ~isempty.
% gjm

    if isempty(a)
       out = 0; 
    else
        out = 1;
    end

end