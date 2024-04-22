
classdef OperationInfo < handle

properties
    
    UAVid     char        % UAV unique ID

    VPsource  char        % id / "unregistered"
    VPdest    char        % id / "unregistered"
    fp        FlightPlan

end
    
methods

function obj = OperationInfo()
    obj.UAVid = '';
    obj.VPsource = 'unregistered';
    obj.VPdest   = 'unregistered';
    obj.fp = FlightPlan.empty;
end


end % methods
end % classdef

