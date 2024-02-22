
classdef USpaceOperator < handle      

properties

    % U-space properties
    name    string              % Operator name
    VPs  = PortInfo.empty;      % list of available vertiports
    UAVs = UAVinfo.empty;       % list of managed UAVs
    ops  = OperationInfo.empty; % list of generated operations

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
        '/NavSim/Time','builtin_interfaces/Time');

    obj.rosCli_SimControl = ros2svcclient(obj.rosNode, ...
        '/NavSim/SimControl','navsim_msgs/SimControl', ...
        'History','keepall');

    obj.rosCli_DeployUAV = ros2svcclient(obj.rosNode, ...
        '/NavSim/DeployModel','navsim_msgs/DeployModel', ...
        'History','keepall');

    obj.rosCli_RemoveUAV = ros2svcclient(obj.rosNode, ...
        '/NavSim/RemoveModel','navsim_msgs/RemoveModel', ...
        'History','keepall');

end


function time = GetTime(obj)
    [msg,status,~] = receive(obj.rosSub_Time,1);
    if (status)
        time = double(msg.sec) + double(msg.nanosec)/1E9;
    else
        time = 0;
    end
end


function WaitTime(obj,time)
    [s,~] = obj.GetTime;
    while s < time
        [s,~] = obj.GetTime;
        % pause (1);

    end
end


function status = PauseSim(obj)

    % Call ROS2 service
    req = ros2message(obj.rosCli_SimControl);
    req.reset = false;
    req.pause = true;

    status = waitForServer(obj.rosCli_SimControl,'Timeout',1);
    if status
        try
            call(obj.rosCli_SimControl,req,'Timeout',1);
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

    status = waitForServer(obj.rosCli_SimControl,'Timeout',1);
    if status
        try
            call(obj.rosCli_SimControl,req,'Timeout',1);
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

    port = PortInfo(id,pos);
    obj.VPs = [obj.VPs port];

    status = true;
end


function status = DeleteVertiport(obj,id)

    i = obj.GetPORTindex(id);
    if i == -1
        status = false;
        return
    end
    obj.VPs = [ obj.VPs(1:i-1) obj.VPs(i+1:end) ];
    status = true;

end


function index = GetPORTindex(obj,id)
    index = -1;
    l = length(obj.VPs);
    if l==0
        return
    end
    for i = 1:l
        if obj.VPs(i).id == id
            index = i;
            return
        end
    end
end


function DeployFleet(obj,numUAVs,model)
    if numUAVs > length(obj.VPs)
        disp("Cannot generate more drones than available vertiports")
        return
    end

    for i = 1:numUAVs
  
        id = sprintf('UAV%02d', i);
        obj.DeployUAV(model,id, ...
            obj.VPs(i).pos+[0 0 0.10], ...
            [0 0 rand*2*pi]);       
    end
end


function UAVids = Fleet(obj)
    % the operator provides a list with the id of all its drones
    numUAVs = length(obj.UAVs);
    UAVids = strings(1,numUAVs);
    for i = 1:numUAVs
        UAVids(i) = obj.UAVs(i).id;
    end
end


function status = DeployUAV(obj,model,UAVid,pos,rot)

    status = false;

    if obj.GetUAVindex(UAVid) ~= -1
        return
    end

    uav = UAVinfo(UAVid,model);

    switch model

        case UAVmodels.MiniDroneCommanded

            file = fullfile(obj.models_path,'/UAM/minidrone/model.sdf');

            uav.rosPub_RemoteCommand = ros2publisher(obj.rosNode, ...
                ['/NavSim/' UAVid '/RemoteCommand'],      ...
                'navsim_msgs/RemoteCommand');

        case UAVmodels.MiniDroneFP1

            file = fullfile(obj.models_path,'/UAM/minidrone/model_FP1.sdf');

            uav.rosPub_FlightPlan = ros2publisher(obj.rosNode, ...
                ['/NavSim/' UAVid '/FlightPlan'],      ...
                'navsim_msgs/FlightPlan');

            uav.rosSub_NavigationReport = ros2subscriber(obj.rosNode, ...
                ['/NavSim/' UAVid '/NavigationReport'], ...
                'navsim_msgs/NavigationReport', ...
                @obj.NavigationReportCallback);

            uav.rosSub_Telemetry = ros2subscriber(obj.rosNode, ...
                ['/NavSim/' UAVid '/Telemetry'], ...
                'navsim_msgs/Telemetry', ...
                @obj.TelemetryCallback);

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

    status = waitForServer(obj.rosCli_DeployUAV,'Timeout',1);
    if status
        try
            call(obj.rosCli_DeployUAV,req,'Timeout',1);
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

    uav = obj.UAVs(i);
    obj.UAVs = [ obj.UAVs(1:i-1) obj.UAVs(i+1:end) ];

    req = ros2message(obj.rosCli_RemoveUAV);
    req.name  = uav.id;  

    status = waitForServer(obj.rosCli_RemoveUAV,'Timeout',1);
    if status
        try
            call(obj.rosCli_RemoveUAV,req,'Timeout',1);
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

    uav = obj.UAVs(i);
    if uav.model ~= UAVmodels.MiniDroneCommanded
        return
    end

    msg = ros2message(uav.rosPub_RemoteCommand);
    msg.uav_id        = uav.id;
    msg.on            = on;
    msg.vel.linear.x  = velX;
    msg.vel.linear.y  = velY;
    msg.vel.linear.z  = velZ;
    msg.vel.angular.z = rotZ;
    msg.duration.sec  = int32(duration);
    send(UAV.rosPub_RemoteCommand,msg);
        
end


function OperateUAV(obj, UAVid)
    % the operator generate an operation for an idle UAV
    
    i = obj.GetUAVindex(UAVid); 
    if  i == -1
        return
    end

    fprintf("%s assigning an operation to %s \n",obj.name,UAVid);

    op = OperationInfo;
    op.uav_id = UAVid;

    % VAMOS POR AQUI

    way_data1 = [ 05   -190.00  -119.00  +048.05   
                  10   -190.00  -119.00  +052.00   
                  30   -190.00  -159.00  +052.00   
                  50   -150.00  -159.00  +052.00   
                  70   -150.00  -119.00  +052.00   
                  90   -190.00  -119.00  +052.00   
                  95   -190.00  -119.00  +048.05  
                  97   -190.00  -119.00  +048.05  ];
    

    fp  = FlightPlan(1,Waypoint.empty);
    
    for j = 1:size(way_data1,1)
        wp = Waypoint();
        wp.t = way_data1(j,1);
        wp.SetPosition(way_data1(j,2:4));
        fp.SetWaypoint(wp);
    end

    time = obj.GetTime();
    fp.RescheduleAt(time + 5);

    op.VPsource = 'VP1';
    op.VPdest   = 'VP2';
    op.fp = fp;
    obj.ops = [obj.ops op];
    obj.UAVs(i).op = length(obj.ops);
    obj.SendFlightPlan('UAV01',fp);

end


function GenerateFlightPlan(obj,VPsource,VPdest)
    
end


function SendFlightPlan(obj,UAVid,fp)
    % fp es una lista de filas con 7 componentes

    i = obj.GetUAVindex(UAVid);
    if i == -1
        return
    end

    % Check drone / FP compatibility
    uav = obj.UAVs(i);
    if uav.model == UAVmodels.MiniDroneFP1
        if fp.mode == InterpolationModes.TP
        else
            return
        end
    else
        return
    end

    % Send ROS2 message
    msg = ros2message(uav.rosPub_FlightPlan);
    msg.plan_id     = uint16(fp.id);
    msg.uav_id      = char(uav.id);
    msg.operator_id = char(obj.name);
    msg.priority    = int8(fp.priority);
    msg.mode        = char(fp.mode);
    msg.radius      = fp.radius;

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
    send(uav.rosPub_FlightPlan,msg);
        
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



function TelemetryCallback(obj,msg)

    i = obj.GetUAVindex(msg.uav_id);

    if i == -1
        return
    end

    obj.UAVs(i).time = double(msg.time.sec) + double(msg.time.nanosec)/1E9;
    obj.UAVs(i).pos = [msg.pose.position.x   msg.pose.position.y   msg.pose.position.z];
    obj.UAVs(i).vel = [msg.velocity.linear.x msg.velocity.linear.y msg.velocity.linear.z];

end



function NavigationReportCallback(obj,msg)

    % disp("NavigationReport received")

    if msg.operator_id ~= obj.name
        fprintf("WARNING: Operator '%s' is managing a UAV that he does not own \n\n",obj.name);
        return
    end
    
    if msg.fp_aborted
        fprintf("%s has discarded the proposed flight plan \n",msg.uav_id);
    end

    if msg.fp_running
        if (msg.current_wp == 0)
            fprintf("%s waiting at starting WP0 \n",msg.uav_id);
        elseif (msg.current_wp == 1)
            fprintf("%s starting flight to WP1 \n",msg.uav_id);
        else
            fprintf("%s heading WP%d \n",msg.uav_id,msg.current_wp);
        end
    end

    if msg.fp_completed
        fprintf("%s has completed its flight plan \n",msg.uav_id);
        % i = obj.GetUAVindex(msg.uav_id);
        % uav = obj.UAVs(i);
        % fp = uav.fp;
        
    end

    % time = double(msg.time.sec) + double(msg.time.nanosec)/1E9;

end



end % methods 
end % classdef

