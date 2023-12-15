%Class for a UAV in the simulation

classdef UAVinfo < handle

properties

drone_id    string         % Unique ID (provided by the U-space registry service)
model       string
operator_id string         % Operator ID of the drone

init_loc                   % Vector3<double>: Spawn location of the drone in the world 

%ROS
rosPub_RemoteCommand
% rosSub_Telemetry         % ROS2 Subscriber object reference

end

methods

%Constructor of the class
function obj = UAVinfo(operator_id, model, init_loc)
    obj.operator_id = operator_id;
    obj.model = model;
    obj.init_loc = init_loc;
    obj.status = 0;
    % obj.rosPub_RemoteCommand = ros2publisher(sprintf('/drone/%d/uplan', obj.drone_id),"utrafman/Uplan");
end


end
end

