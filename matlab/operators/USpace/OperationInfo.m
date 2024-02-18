
classdef OperationInfo < handle

properties
    
    VPsource  string      % id / "unregistered"
    VPdest    string      % id / "unregistered"
    fp        FlightPlan
    uav_id    string      % UAV unique ID

end
    
methods

function obj = OperationInfo(id, pos)
    obj.VPsource = "unregistered";
    obj.VPdest   = "unregistered";
    obj.fp = FlightPlan.empty;
    obj.uav_id = '';
end


end % methods
end % classdef

