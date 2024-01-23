
classdef USpaceOperator < handle      

properties

    % U-space properties
    name    string            % Operator name
    ports = struct([])        % list of vertiports
    UAVs  = struct([])        % list of UAVs fleet

    % ROS2 interface
    rosNode                   % ROS2 Node 
    rosSub_Time               % ROS2 subscriptor to get simulation time
    rosCli_SimControl         % ROS2 Service client to control the simulation
    rosCli_DeployUAV          % ROS2 Service client to deploy models into the air space
    rosCli_RemoveUAV          % ROS2 Service client to remove models from the air space

    % Gazebo interface
    models_path               

end

methods


function obj = USpaceOperator(name,models_path)

    obj.name = name;
    obj.models_path = models_path;

    % ROS2 node
    obj.rosNode = ros2node(obj.name);

    obj.rosSub_Time = ros2subscriber(obj.rosNode, ...
        '/World/Time','builtin_interfaces/Time');

    obj.rosCli_SimControl = ros2svcclient(obj.rosNode, ...
        '/World/SimControl','navsim_msgs/SimControl', ...
        'History','keepall');

    obj.rosCli_DeployUAV = ros2svcclient(obj.rosNode, ...
        '/World/DeployModel','navsim_msgs/DeployModel', ...
        'History','keepall');

    obj.rosCli_RemoveUAV = ros2svcclient(obj.rosNode, ...
        '/World/RemoveModel','navsim_msgs/RemoveModel', ...
        'History','keepall');

end


function [sec,mil] = GetTime(obj)
    [msg,status,~] = receive(obj.rosSub_Time,1);
    if (status)
        sec = double(msg.sec);
        mil = double(msg.nanosec / 1E6);
    else
        sec = 0;
        mil = 0;
    end
end


function WaitTime(obj,time)
    [s,~] = obj.GetTime;
    while s < time
        [s,~] = obj.GetTime;
        pause (1);
    end
end


function status = PauseSim(obj)

    % Call ROS2 service
    req = ros2message(obj.rosCli_SimControl);
    req.reset = false;
    req.pause = true;

    status = waitForServer(obj.rosCli_SimControl,"Timeout",1);
    if status
        try
            call(obj.rosCli_SimControl,req,"Timeout",1);
        catch
            status = false;
        end
    end
    
end


function status = ResetSim(obj)

    % Call ROS2 service
    req = ros2message(obj.rosCli_SimControl);
    req.reset = true;
    req.pause = false;

    status = waitForServer(obj.rosCli_SimControl,"Timeout",1);
    if status
        try
            call(obj.rosCli_SimControl,req,"Timeout",1);
        catch
            status = false;
        end
    end
    
end


function status = SetVertiport(obj,id,pos)

    if obj.GetPORTindex(id) ~= -1
        status = false;
        return
    end

    port.id = id;
    port.pos = pos;

    obj.ports = [obj.ports port];
    status = true;
end


function status = DeleteVertiport(obj,id)

    i = obj.GetPORTindex(id);
    if i == -1
        status = false;
        return
    end
    obj.ports = [ obj.ports(1:i-1) obj.ports(i+1:end) ];
    status = true;

end


function index = GetPORTindex(obj,id)
    index = -1;
    l = length(obj.ports);
    if l==0
        return
    end
    for i = 1:l
        if obj.ports(i).id == id
            index = i;
            return
        end
    end
end


function status = DeployUAV(obj,model,UAVid,pos,rot)

    status = false;

    if obj.GetUAVindex(UAVid) ~= -1
        return
    end

    uav.id = UAVid;
    uav.model = model;

    switch model
        case UAVmodels.MiniDroneCommanded
            file = fullfile(obj.models_path,'/UAM/minidrone/model.sdf');
            uav.rosPub = ros2publisher(obj.rosNode, ...
                ['/UAV/',UAVid,'/RemoteCommand'],      ...
                "navsim_msgs/RemoteCommand");

        case UAVmodels.MiniDroneFP1
            file = fullfile(obj.models_path,'/UAM/minidrone/model_FP1.sdf');
            uav.rosPub = ros2publisher(obj.rosNode, ...
                ['/UAV/',UAVid,'/FlightPlan'],      ...
                "navsim_msgs/FlightPlan");
        otherwise
            return
    end

    obj.UAVs = [obj.UAVs uav];

    % Call ROS2 service
    req = ros2message(obj.rosCli_DeployUAV);
    req.model_sdf = fileread(file);
    req.name  = UAVid; 
    req.pos.x = pos(1);
    req.pos.y = pos(2);
    req.pos.z = pos(3);
    req.rot.x = rot(1);
    req.rot.y = rot(2);
    req.rot.z = rot(3);

    status = waitForServer(obj.rosCli_DeployUAV,"Timeout",1);
    if status
        try
            call(obj.rosCli_DeployUAV,req,"Timeout",1);
        catch
            status = false;
        end
    end


end


function status = RemoveUAV(obj,id)

    i = obj.GetUAVindex(id);

    if i == -1
        return
    end

    UAV = obj.UAVs(i);
    obj.UAVs = [ obj.UAVs(1:i-1) obj.UAVs(i+1:end) ];

    req = ros2message(obj.rosCli_RemoveUAV);
    req.name  = UAV.id;  

    status = waitForServer(obj.rosCli_RemoveUAV,"Timeout",1);
    if status
        try
            call(obj.rosCli_RemoveUAV,req,"Timeout",1);
        catch
            status = false;
        end
    end
    

end


function RemoteCommand(obj,UAVid,on,velX,velY,velZ,rotZ,duration)

    i = obj.GetUAVindex(UAVid);

    if i == -1
        return
    end

    UAV = obj.UAVs(i);
    if UAV.model ~= UAVmodels.MiniDroneCommanded
        return
    end

    msg = ros2message(UAV.rosPub);
    msg.uav_id        = UAV.id;
    msg.on            = on;
    msg.vel.linear.x  = velX;
    msg.vel.linear.y  = velY;
    msg.vel.linear.z  = velZ;
    msg.vel.angular.z = rotZ;
    msg.duration.sec  = int32(duration);
    send(UAV.rosPub,msg);
        
end


function SendFlightPlan(obj,UAVid,fp)
    % fp es una lista de filas con 7 componentes

    i = obj.GetUAVindex(UAVid);

    if i == -1
        return
    end

    % Check drone / FP compatibility
    UAV = obj.UAVs(i);
    if UAV.model == UAVmodels.MiniDroneFP1
        if fp.mode == InterpolationModes.TP
        else
            return
        end
    else
        return
    end

    msg = ros2message(UAV.rosPub);
    msg.plan_id     = uint16(fp.id);
    msg.uav_id      = UAV.id;
    msg.operator_id = char(obj.name);
    msg.priority    = int8(fp.priority);

    for i = 1:length(fp.waypoints)
        t = fp.waypoints(i).t;
        msg.route(i).time.sec = int32(floor(t));
        msg.route(i).time.nanosec = uint32(rem(t,1)*1E9);

        msg.route(i).pos.x = fp.waypoints(i).x;
        msg.route(i).pos.y = fp.waypoints(i).y;
        msg.route(i).pos.z = fp.waypoints(i).z;

        msg.route(i).vel.x = fp.waypoints(i).vx;
        msg.route(i).vel.y = fp.waypoints(i).vy;
        msg.route(i).vel.z = fp.waypoints(i).vz;

    end
    send(UAV.rosPub,msg);
        
end


function index = GetUAVindex(obj,id)
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


% Destructor
function delete(obj)
    % Cerrar y eliminar los recursos ROS2
    if ~isempty(obj.rosCli_RemoveUAV)
        delete(obj.rosCli_RemoveUAV);
    end
    
    if ~isempty(obj.rosCli_DeployUAV)
        delete(obj.rosCli_DeployUAV);
    end
    
    if ~isempty(obj.rosCli_SimControl)
        delete(obj.rosCli_SimControl);
    end
    
    if ~isempty(obj.rosSub_Time)
        delete(obj.rosSub_Time);
    end
    
    if ~isempty(obj.rosNode)
        delete(obj.rosNode);
    end
    disp('ROS2 operator deleted')
end



end % methods 
end % classdef

