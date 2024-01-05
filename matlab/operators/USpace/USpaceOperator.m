% Operator classes represent operators in the context of U-space.
% Each operator has a drone garage where it stores its UAVs.

classdef USpaceOperator < handle      

properties
    name    string            % Operator name
    models_path               

    fleet   UAVinfo           % Array of UAV objects references

    % ROS2 interface
    rosNode                        % ROS2 Node 
    rosCli_Time                    % ROS2 service client 
    rosCli_DeployUAV               % ROS2 Service client to deploy models into the air space
    rosCli_RemoveUAV               % ROS2 Service client to remove models from the air space
    % ROScli_reg_operator          % Service client to register itself as operators
    % ROScli_reg_FP                % Service client to register a new FP

end

methods


%Class constructor
function obj = USpaceOperator(name,models_path)

    obj.name = name;
    obj.models_path = models_path;

    % ROS2 node
    obj.rosNode = ros2node(obj.name);

    obj.rosCli_Time = ros2svcclient(obj.rosNode, ...
        '/World/Time','navsim_msgs/Time');

    obj.rosCli_DeployUAV = ros2svcclient(obj.rosNode, ...
        '/World/DeployModel','navsim_msgs/DeployModel', ...
        'History','keepall');

    obj.rosCli_RemoveUAV = ros2svcclient(obj.rosNode, ...
        '/World/RemoveModel','navsim_msgs/RemoveModel', ...
        'History','keepall');

end


function [sec,mil] = GetTime(obj)
    req = ros2message(obj.rosCli_Time);
    req.reset = false;
    status = waitForServer(obj.rosCli_Time,"Timeout",1);
    if ~status
        error("Es servicio ROS2 no está disponible")
    end
    res = call(obj.rosCli_Time,req,"Timeout",1);

    sec = res.time.sec;
    mil = res.time.nanosec / 1E6;
end


function ResetTime(obj)
    req = ros2message(obj.rosCli_Time);
    req.reset = true;
    status = waitForServer(obj.rosCli_Time,"Timeout",1);
    if ~status
        error("ROS2 service is not available")
    end
    call(obj.rosCli_Time,req,"Timeout",1);
end



function status = DeployUAV(obj,model,name,pos,rot)

    switch model
        case UAVmodels.MiniDroneCommanded
            file = fullfile(obj.models_path,'/UAM/minidrone/model.sdf');
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

