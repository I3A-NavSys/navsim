%Class for a UAV in the simulation

classdef PortInfo < handle

properties
    id   string      % Port unique ID
    pos              % Port 3D position
    radius           % A port is assumed to be a circle
end
    
methods

function obj = PortInfo(id, pos, radius)
    obj.id     = id;
    obj.pos    = pos;
    obj.radius = radius;
end


end % methods
end % classdef

