%Class for a UAV in the simulation

classdef UAVinfo < handle

properties

name        string         % Unique ID (provided by the U-space registry service)
model       string
operator    string         % Operator ID of the drone

%ROS
rosPub_RemoteCommand       % publisher to remotely pilot a drone
% rosSub_Telemetry         % ROS2 Subscriber object reference

end

methods

%Constructor of the class
function obj = UAVinfo(name, model, operator)
    obj.name = name;
    obj.model = model;
    obj.operator = operator.name;

    if model == UAVmodels.MiniDroneCommanded
        obj.rosPub_RemoteCommand = ros2publisher(operator.rosNode, ...
            ['/UAV/',name,'/RemoteCommand'],      ...
            "navsim_msgs/RemoteCommand");





    end

end

end %methods
end %classdef


   