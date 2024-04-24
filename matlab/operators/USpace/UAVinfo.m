
classdef UAVinfo < handle

properties
    % Aircraft information
    id         char       % UAV unique ID
    model      UAVmodels

    % Aircraft performance
    maxForwardVel   = 0    % cruising_speed
    maxForwardAcel  = 0
    maxVerticalVel  = 0  
    maxVerticalAcel = 0
    maxAngularVel   = 0  
    maxAngularAcel  = 0

    % Telemetry information
    time = 0
    pos  = 0
    vel  = 0

    % Current operation number
    op = 0                     
    
    % ROS2 interface
    rosPub_RemoteCommand
    rosPub_FlightPlan
    rosSub_NavigationReport
    rosSub_Telemetry

end
    
methods

function obj = UAVinfo(id, model)
    obj.id    = char(id);
    obj.model = model;

    obj.rosPub_RemoteCommand = ros2publisher.empty;
    obj.rosPub_FlightPlan = ros2publisher.empty;
    obj.rosSub_NavigationReport = ros2subscriber.empty;
    obj.rosSub_Telemetry = ros2subscriber.empty;
end


end % methods
end % classdef

