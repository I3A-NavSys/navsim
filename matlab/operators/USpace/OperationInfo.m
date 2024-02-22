
classdef OperationInfo < handle

properties
    
    uav_id    string      % UAV unique ID

    VPsource  string      % id / "unregistered"
    VPdest    string      % id / "unregistered"
    fp        FlightPlan

end
    
methods

function obj = OperationInfo()
    obj.VPsource = "unregistered";
    obj.VPdest   = "unregistered";
    obj.fp = FlightPlan.empty;
    obj.uav_id = '';
end


end % methods
end % classdef

