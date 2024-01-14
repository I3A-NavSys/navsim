classdef SimpleMonitor < handle

properties
    name    string            % Monitor name
    UAVs = struct([])         % list of UAVs being tracked

    % ROS2 interface
    rosNode                   % ROS2 Node 

end

methods


function obj = SimpleMonitor(name)

    obj.name = name;


    % ROS2 node
    obj.rosNode = ros2node(obj.name);

end


function trackUAV(obj,UAVid)

    if obj.getUAVindex(UAVid) ~= -1
        return
    end

    uav.id = UAVid;
    uav.data = double.empty(0,7);
    uav.rosSub_Telemetry = ros2subscriber(obj.rosNode, ...
        ['/UAV/' UAVid '/Telemetry'],'navsim_msgs/Telemetry', ...
        @obj.TelemetryCallback);

    obj.UAVs = [obj.UAVs uav];


end



function TelemetryViewer(obj,UAVid)

    i = obj.getUAVindex(UAVid);
    if i == -1
        return
    end

    t = obj.UAVs(i).data(:,1);
    x = obj.UAVs(i).data(:,2);
    y = obj.UAVs(i).data(:,3);
    z = obj.UAVs(i).data(:,4);


    

    % Checking figure
    figName = [UAVid ' Telemetry'];
    figHandler = findobj('Type','figure','Name',figName)';
    if (isempty(figHandler)) 
        figHandler = figure( ...
            'Name',figName, ...
            'NumberTitle','off', ...
            'Position',[200 200 800 600]);
    else
        clf(figHandler)
    end


    %3D viewer
    subplot(6,2,[1 11])
    plot3(x,y,z,".:",'Color','black','MarkerSize',10)
    grid on
    % plot3(ux(1:end-1),uy(1:end-1),uz(1:end-1));
    % plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'or');
    % legend(["Simulated", "Reference", "Waypoints"],'Location','northwest');
    title("Route 3D view")
    xlabel("pos X")
    ylabel("pos Y")
    zlabel("pos Z")

    hold on


    %XYZ pos
    subplot(6,2,[2 4]);
    plot(t,x,".:",'Color','black','MarkerSize',10)
    grid on

    subplot(6,2,[6 8]);
    plot(t,y,".:",'Color','black','MarkerSize',10)
    grid on

    subplot(6,2,[10 12]);
    plot(t,z,".:",'Color','black','MarkerSize',10)
    grid on



end



function index = getUAVindex(obj,id)
    index = -1;
    l = length(obj.UAVs);
    if l==0
        return
    end
    for i = 1:l
        if obj.UAVs(i).id == id
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
    if i == -1
        return
    end

    if  ~isempty(obj.UAVs(i).data)
        if  (time < obj.UAVs(i).data(end,1))
            obj.UAVs(i).data = double.empty(0,7);
        end
    end
    obj.UAVs(i).data(end+1,:) = [time pos vel];

end


end % methods
end % classdef