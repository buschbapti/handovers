function T = se3(varargin)
% function T = se3(roll, pitch, yaw, x, y, z)
% OR
% function T = se3([roll, pitch, yaw, x, y, z])
%
% Angles are given in radians.
%
% Provides a pose transformation.
% The rotation matrix R uses RPY order in intrinsic rotations
% You can use tr2rpy to recover the angles back.
%

if nargin == 1
    roll  = varargin{1}(1);
    pitch = varargin{1}(2);
    yaw   = varargin{1}(3);
    x     = varargin{1}(4);
    y     = varargin{1}(5);
    z     = varargin{1}(6);
elseif nargin == 6
    roll  = varargin{1};
    pitch = varargin{2};
    yaw   = varargin{3};
    x     = varargin{4};
    y     = varargin{5};
    z     = varargin{6};
end


T = [
  cos(pitch)*cos(yaw),                 -cos(pitch)*sin(yaw),       sin(pitch), x
  cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll), cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw), -cos(pitch)*sin(roll), y
  sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch), cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw),  cos(pitch)*cos(roll), z
  0 0 0 1
    ];


end
