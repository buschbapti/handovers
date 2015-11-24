function name_with_time = my_time( str )
% function name_with_time = my_time( str )
% Return a string with the current date in the format 20150120_183422
%
% INPUT
%   str: an optional string that if given will be attached at the end of
%        the date. Otherwise only the date will be returned.
%   

    if exist('str', 'var')
        name_with_time = [datestr(now, 'yyyymmdd_HHMMSS')  str];
    else
        name_with_time = [datestr(now, 'yyyymmdd_HHMMSS') ];
    end
    
end