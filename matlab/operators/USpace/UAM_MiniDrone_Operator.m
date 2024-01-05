% Operator classes represent operators in the context of U-space.
% Each operator has a drone garage where it stores its UAVs.

classdef UAM_MiniDrone_Operator < handle      % Drone Challenge operator

properties
    name string                    % Operator name
    id   uint16                    % Unique ID (provided by the U-space registry service)
    file string                    % path to SDF model
    % drone_garage UAVProperties     % Array of UAV objects references

    % ROS2 interface
    rosNode                        % ROS2 Node 
    rosCli_Time                    % ROS2 service client 
    rosCli_DeployModel             % ROS2 Service client to deploy models into the air space
    rosCli_RemoveModel             % ROS2 Service client to remove models from the air space
    % ROScli_reg_operator          % Service client to register itself as operators
    % ROScli_reg_FP                % Service client to register a new FP

end

methods


%Class constructor
function obj = UAM_MiniDrone_Operator(name,path)

    obj.name = name;
    obj.file = fullfile(path,'/UAM/minidrone/minidrone_cmd.sdf');
    
    % ROS2
    obj.rosNode = ros2node(obj.name);

    obj.rosCli_Time = ros2svcclient(obj.rosNode, ...
        '/World/Time','navsim_msgs/Time');
    
    obj.rosCli_DeployModel = ros2svcclient(obj.rosNode, ...
        '/World/DeployModel','navsim_msgs/DeployModel', ...
        'History','keepall');
    
    obj.rosCli_RemoveModel = ros2svcclient(obj.rosNode, ...
        '/World/RemoveModel','navsim_msgs/RemoveModel', ...
        'History','keepall');

    obj.rosPub_RemoteCommand = ros2publisher(obj.rosNode, ...
        '/UAV/abejorro1/RemoteCommand', ...
        'navsim_msgs/RemoteCommand', ...
        'History','keeplast');

    obj.rosSub_GetImage = ros2subscriber(obj.rosNode, ...
        '/UAV/abejorro1/cam/image_raw','sensor_msgs/Image');
    
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


function status =  DeployUAV(obj,name,pos,rot)

    req = ros2message(obj.rosCli_DeployModel);
    req.model_sdf = fileread(obj.file);
    req.name  = name;  %'deployedModel'
    req.pos.x = pos(1);
    req.pos.y = pos(2);
    req.pos.z = pos(3);
    req.rot.x = rot(1);
    req.rot.y = rot(2);
    req.rot.z = rot(3);

    status = waitForServer(obj.rosCli_DeployModel,"Timeout",1);
    if status
        try
            call(obj.rosCli_DeployModel,req,"Timeout",1);
        catch
            status = false;
        end
    end
        
end


function status =  RemoveUAV(obj,name)

    req = ros2message(obj.rosCli_RemoveUAV);
    req.name  = name;  %'deployedModel'

    status = waitForServer(obj.rosCli_RemoveUAV,"Timeout",1);
    if status
        try
            call(obj.rosCli_RemoveUAV,req,"Timeout",1);
        catch
            status = false;
        end
    end
        
end



function RemoteCommand(obj,UAVid,on,velX,velY,velZ,rotZ)

    msg = ros2message(obj.rosPub_RemoteCommand);
    msg.uav_id        = UAVid;
    msg.on            = on;
    msg.vel.linear.x  = velX;
    msg.vel.linear.y  = velY;
    msg.vel.linear.z  = velZ;
    msg.vel.angular.z = rotZ;
    msg.duration.sec  = int32(1);
    send(obj.rosPub_RemoteCommand,msg);
        
end



end % methods 
end % classdef

