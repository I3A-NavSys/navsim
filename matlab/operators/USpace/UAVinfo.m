%Class for a UAV in the simulation

classdef UAVinfo < handle

properties

name        string         % Unique ID (provided by the U-space registry service)
model       string
operator    string         % Operator ID of the drone

%ROS
rosPub_RemoteCommand       % publisher to remotely pilot a drone
rosPub_FlightPlan          % publisher to send flight plans to the drone
% rosSub_Telemetry         % ROS2 Subscriber object reference

end

methods

%Constructor of the class
function obj = UAVinfo(name, model, operator)
    obj.name = name;
    obj.model = model;
    obj.operator = operator.name;

    switch model
        case UAVmodels.MiniDroneCommanded
            obj.rosPub_RemoteCommand = ros2publisher(operator.rosNode, ...
                ['/UAV/',name,'/RemoteCommand'],      ...
                "navsim_msgs/RemoteCommand");
        case UAVmodels.MiniDroneFP1
            obj.rosPub_FlightPlan = ros2publisher(operator.rosNode, ...
                ['/UAV/',name,'/FlightPlan'],      ...
                "navsim_msgs/FlightPlan");

    end

end


function SendFlightPlan(obj,fp)

end

end %methods
end %classdef


   