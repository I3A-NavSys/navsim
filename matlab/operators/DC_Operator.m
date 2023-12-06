% Operator classes represent operators in the context of U-space.
% Each operator has a drone garage where it stores its UAVs.

classdef DC_Operator < handle      % Drone Challenge operator

properties
    name         string            % Operator name
    id           uint16            % Unique ID (provided by the U-space registry service)
    % drone_garage UAVProperties     % Array of UAV objects references

    % ROS2 interface
    rosNode                        % ROS2 Node 
    rosCli_Test                    % ROS2 service client 
    rosCli_DeployModel             % ROS2 Service client to deploy models into the air space
    % ROScli_reg_operator            % Service client to register itself as operators
    % ROScli_reg_FP                  % Service client to register a new FP

end

methods


%Class constructor
function obj = DC_Operator(name)

    obj.name = name;
    
    % ROS2
    obj.rosNode = ros2node(obj.name);
    obj.rosCli_DeployModel = ros2svcclient(obj.rosNode, ...
        '/World/DeployModel','utrafman_msgs/DeployModel', ...
        'History','keepall');

end


function status =  DeployUAV(obj,name,pos,rot)

    path = '../../utrafman_ws/src/utrafman_pkg/models/';
    file = strcat(path,'DCmodels/drone/model.sdf');

    req = ros2message(obj.rosCli_DeployModel);
    req.model_sdf = fileread(file);
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

end % methods 
end % classdef

