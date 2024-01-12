classdef SimpleMonitor < handle

properties
    name    string            % Monitor name
    uavs = struct([])         % list of UAVs being tracked

    % ROS2 interface
    rosNode                   % ROS2 Node 

end

methods


function obj = SimpleMonitor(name)

    obj.name = name;


    % ROS2 node
    obj.rosNode = ros2node(obj.name);

end


function trackUAV(obj,id)

    if obj.getUAVindex(id) ~= -1
        return
    end

    uav.id = id;
    uav.data = double.empty(0,7);
    uav.rosSub_Telemetry = ros2subscriber(obj.rosNode, ...
        ['/UAV/' id '/Telemetry'],'navsim_msgs/Telemetry', ...
        @obj.TelemetryCallback);

    obj.uavs = [obj.uavs uav];


end

function index = getUAVindex(obj,id)
    index = -1;
    l = length(obj.uavs);
    if l==0
        return
    end
    for i = 1:l
        if obj.uavs(i).id == id
            index = i;
            return
        end
    end
end

function TelemetryCallback(obj,msg)

    time = double(msg.time.sec) + double(msg.time.nanosec)/1E9;
    pos = [msg.pose.position.x   msg.pose.position.y   msg.pose.position.z];
    vel = [msg.velocity.linear.x msg.velocity.linear.y msg.velocity.linear.z];

    i = obj.getUAVindex(msg.uav_id);
    if time < obj.uavs(i).data(end,1)
        obj.uavs(i).data = [time pos vel];
    else
        obj.uavs(i).data(end+1,:) = [time pos vel];
    end

end

end % methods
end % classdef