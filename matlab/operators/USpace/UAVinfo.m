%Class for a UAV in the simulation

classdef UAVinfo < handle

properties
    id         string      % UAV unique ID
    model      UAVmodels

    vertiport  string      % id / "flying" / "unregistered"
    operation              % number of current operation
    fp         FlightPlan

    % ROS2 interface
    rosPub_RemoteCommand
    rosPub_FlightPlan
    rosSub_NavigationReport

end
    
methods

function obj = UAVinfo(id, model)
    obj.id        = id;
    obj.model     = model;
    obj.vertiport = 'unregistered';
    obj.operation = 0;
    obj.fp        = FlightPlan.empty;

    obj.rosPub_RemoteCommand = ros2publisher.empty;
    obj.rosPub_FlightPlan = ros2publisher.empty;
    obj.rosSub_NavigationReport = ros2subscriber.empty;

end


end % methods
end % classdef

