
classdef UAVinfo < handle

properties
    % Aircraft information
    id         string      % UAV unique ID
    model      UAVmodels

    % Aircraft performance
    velMax

    % Telemetry information
    time
    pos
    vel

    % Current operation number
    op                     
    
    % ROS2 interface
    rosPub_RemoteCommand
    rosPub_FlightPlan
    rosSub_NavigationReport
    rosSub_Telemetry

end
    
methods

function obj = UAVinfo(id, model)
    obj.id    = id;
    obj.model = model;
    obj.op    = 0;

    obj.rosPub_RemoteCommand = ros2publisher.empty;
    obj.rosPub_FlightPlan = ros2publisher.empty;
    obj.rosSub_NavigationReport = ros2subscriber.empty;
    obj.rosSub_Telemetry = ros2subscriber.empty;

end


end % methods
end % classdef

