% Builder classes are responsible of populating worlds.

classdef SimpleBuilder < handle

properties
    name string                 % Builder name
    path string                 % path to SDF models

    % ROS2 interface
    rosNode                     % ROS2 Node 
    rosCli_DeployModel          % ROS2 Service client to deploy models into the air space

end

methods


%Class constructor
function obj = SimpleBuilder(name,path)

    obj.name = name;
    obj.path = path;
    
    % ROS2 node
    obj.rosNode = ros2node(obj.name);


    % ROS2 service clients
    obj.rosCli_DeployModel = ros2svcclient(obj.rosNode, ...
        '/World/DeployModel','navsim_msgs/DeployModel', ...
        'History','keepall');

end


function status =  DeployModel(obj,model,name,pos,rot)

    file = fullfile(obj.path,model,'/model.sdf');
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

