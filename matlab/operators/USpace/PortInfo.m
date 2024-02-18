%Class for a UAV in the simulation

classdef PortInfo < handle

properties
    id   string      % Port unique ID
    pos              % Port 3D position
end
    
methods

function obj = PortInfo(id, pos)
    obj.id  = id;
    obj.pos = pos;
end


end % methods
end % classdef

