
classdef USpaceOperator < handle      

properties
    name    string            % Operator name
    models_path               

    fleet   UAVinfo           % Array of UAV objects references

    % ROS2 interface
    rosNode                   % ROS2 Node 
    rosSub_Time               % ROS2 subscriptor to get simulation time
    rosCli_DeployUAV          % ROS2 Service client to deploy models into the air space
    rosCli_RemoveUAV          % ROS2 Service client to remove models from the air space

end

methods


function obj = USpaceOperator(name,models_path)

    obj.name = name;
    obj.models_path = models_path;

    % ROS2 node
    obj.rosNode = ros2node(obj.name);

    obj.rosSub_Time = ros2subscriber(obj.rosNode, ...
        '/World/Time','builtin_interfaces/Time', ...
        'History','keeplast');

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


function status = DeployUAV(obj,model,name,pos,rot)

    switch model
        case UAVmodels.MiniDroneCommanded
            file = fullfile(obj.models_path,'/UAM/minidrone/model.sdf');
        case UAVmodels.MiniDroneFP1
            file = fullfile(obj.models_path,'/UAM/minidrone/model_FP1.sdf');
        otherwise
            status = false;
            return
    end

    req = ros2message(obj.rosCli_DeployUAV);
    req.model_sdf = fileread(file);
    req.name  = name; 
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

    obj.fleet(end+1) = UAVinfo(name,model,obj);


end


function status = RemoveUAV(obj,name)

    req = ros2message(obj.rosCli_RemoveUAV);
    req.name  = name;  

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

    UAV = obj.GetUAV(UAVid);

    if UAV == false 
        return
    end

    if UAV.model ~= UAVmodels.MiniDroneCommanded
        return
    end

    msg = ros2message(UAV.rosPub_RemoteCommand);
    msg.uav_id        = UAVid;
    msg.on            = on;
    msg.vel.linear.x  = velX;
    msg.vel.linear.y  = velY;
    msg.vel.linear.z  = velZ;
    msg.vel.angular.z = rotZ;
    msg.duration.sec  = int32(duration);
    send(UAV.rosPub_RemoteCommand,msg);
        
end


function SendFlightPlan(obj,UAVid,fp)
    % fp es una lista de filas con 7 componentes

    UAV = obj.GetUAV(UAVid);

    if UAV == false 
        return
    end

    if UAV.model ~= UAVmodels.MiniDroneFP1
        return
    end

    msg = ros2message(UAV.rosPub_FlightPlan);
    msg.plan_id       = uint16(1);  %de momento
    msg.uav_id        = UAVid;
    msg.operator_id   = char(obj.name);

    for i = 1:length(fp(:,7))
        msg.route(i).pos.x = fp(i,1);
        msg.route(i).pos.y = fp(i,2);
        msg.route(i).pos.z = fp(i,3);
        msg.route(i).vel.x = fp(i,4);
        msg.route(i).vel.y = fp(i,5);
        msg.route(i).vel.z = fp(i,6);
        msg.route(i).time.sec = int32(fp(i,7));
        msg.route(i).time.nanosec = uint32(0);
    end
    send(UAV.rosPub_FlightPlan,msg);
        
end


function UAV = GetUAV(obj,name)
    for i = 1:length(obj.fleet)
        if obj.fleet(i).name == name
            UAV = obj.fleet(i);
            return
        end
    end
    UAV = false;
end


end % methods 
end % classdef

