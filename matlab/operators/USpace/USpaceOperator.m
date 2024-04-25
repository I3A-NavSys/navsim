
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
    t = obj.GetTime;
    while t < time
        t = obj.GetTime;
        pause(0.1);
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



function status = SetVertiport(obj,id,pos,radius)

    if obj.GetPORTindex(id) ~= -1
        status = false;
        return
    end

    port = PortInfo(id,pos,radius);
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
        if strcmp(obj.VPs(i).id, id)
            index = i;
            return
        end
    end
end



function DeployFleet(obj,numUAVs,info)
    if numUAVs > length(obj.VPs)
        disp("Cannot generate more drones than available vertiports")
        return
    end

    randomVPs = randperm(length(obj.VPs));
  
    for i = 1:numUAVs
        id = sprintf('UAV%02d', i);
        vp = randomVPs(i);
        obj.DeployUAV(info,id, ...
            obj.VPs(vp).pos+[0 0 0.10], ...
            [0 0 rand*2*pi]);
    end
end



function UAVids = FleetIds(obj)
    % the operator provides a list with the id of all its drones
    numUAVs = length(obj.UAVs);
    UAVids = strings(1,numUAVs);
    for i = 1:numUAVs
        UAVids(i) = obj.UAVs(i).id;
    end
end



function status = DeployUAV(obj,info,UAVid,pos,rot)

    status = false;

    if obj.GetUAVindex(UAVid) ~= -1
        return
    end

    uav = UAVinfo(UAVid,info.model);
    uav.pos = pos;
    uav.maxForwardVel = info.maxForwardVel;

    switch info.model

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

        case UAVmodels.AeroTaxiCommanded  

            file = fullfile(obj.models_path,'/UAM/aerotaxi/model.sdf');

            uav.rosPub_RemoteCommand = ros2publisher(obj.rosNode, ...
                ['/NavSim/' UAVid '/RemoteCommand'],      ...
                'navsim_msgs/RemoteCommand');
            
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
    if uav.model ~= UAVmodels.MiniDroneCommanded  &&  uav.model ~= UAVmodels.AeroTaxiCommanded
        return
    end

    msg = ros2message(uav.rosPub_RemoteCommand);
    msg.uav_id        = char(uav.id);
    msg.on            = on;
    msg.vel.linear.x  = velX;
    msg.vel.linear.y  = velY;
    msg.vel.linear.z  = velZ;
    msg.vel.angular.z = rotZ;
    msg.duration.sec  = int32(duration);
    send(uav.rosPub_RemoteCommand,msg);
        
end



function VPid = GetUAVLocation(obj,UAVid)
    i = obj.GetUAVindex(UAVid); 
    if  i == -1
        return
    end
    
    pos = obj.UAVs(i).pos;
    
    for v = obj.VPs
        if norm(v.pos - pos) < v.radius
            VPid = v.id;
            return;
        end
    end
    VPid = char('unregistered');
end



function OperateUAV(obj, UAVid)
    % the operator generate an operation for an idle UAV.
    % the operation starts in 10 seconds
    
    i = obj.GetUAVindex(UAVid); 
    if  i == -1
        return
    end

    op = OperationInfo;
    op.UAVid = char(UAVid);

    % identifica el vertipuerto actual del dron
    op.VPsource = obj.GetUAVLocation(UAVid);

    if op.VPsource == "unregistered"
        fprintf("\n%s is LOST \n\n",UAVid);
        return;
    end

    % elige vertipuerto de destino
    op.VPdest = op.VPsource;
    while op.VPdest == op.VPsource
        v2 = randi(length(obj.VPs));
        op.VPdest = obj.VPs(v2).id;
    end

    % Generating a flight plan
    op.fp = obj.GenerateFlightPlan(op);
    time = obj.GetTime();
    op.fp.RescheduleAt(time + 10);
    op.fp.id = length(obj.ops)+1;

    % Confirm the operation
    obj.UAVs(i).op = op.fp.id;
    obj.ops = [obj.ops op];
    obj.SendFlightPlan(UAVid,op.fp);
    fprintf("Operation assigned to %s \n",UAVid);

end



function fp = GenerateFlightPlan(obj,op)

    % Get UAV performance
    uav = obj.UAVs(obj.GetUAVindex(op.UAVid));
    
    % Get position of both vertiports
    vp1 = obj.VPs(obj.GetPORTindex(op.VPsource)).pos;
    vp2 = obj.VPs(obj.GetPORTindex(op.VPdest)).pos;
    
    % Create flight plan with 5 waypoints
    fp  = FlightPlan(Waypoint.empty);
    wp1H = Waypoint();
    wp1M = Waypoint();
    wp1L = Waypoint();
    wp3  = Waypoint();
    wp2H = Waypoint();
    wp2M = Waypoint();
    wp2L = Waypoint();
    
    % take off / landing positions
    wp1L.SetPosition(vp1+[0 0 0.25]);
    wp2L.SetPosition(vp2+[0 0 0.25]);

    % take off / landing approach positions
    wp1M.SetPosition(vp1+[0 0 1]);
    wp2M.SetPosition(vp2+[0 0 1]);

    % take off / landing hovering positions
    angle = wp1L.CourseTo(wp2L);
    wp1H.SetPosition(wp1L.Position);
    wp1H.z = 70 + angle/20;
    wp2H.SetPosition(wp2L.Position);
    wp2H.z = wp1H.z;

    wp1H.SetPosition(wp1H.Position + 2 * wp1H.DirectionTo(wp2H));
    wp3.SetPosition(wp2H.Position - 10 * wp1H.DirectionTo(wp2H));

    % waypoint time intervals
    wp1L.t = 0;
    wp1M.t = wp1L.t + wp1L.DistanceTo(wp1M) / 0.2;             % 0.2m/s
    wp1H.t = wp1M.t + wp1M.DistanceTo(wp1H) / 2;               % 2m/s
    wp3.t  = wp1H.t + wp1H.DistanceTo(wp3)  / uav.maxForwardVel;
    wp2H.t = wp3.t  +  wp3.DistanceTo(wp2H) / 2;               % 2m/s
    wp2M.t = wp2H.t + wp2H.DistanceTo(wp2M) / 2;               % 2m/s
    wp2L.t = wp2M.t + wp2M.DistanceTo(wp2L) / 0.2;             % 0.2m/s

    fp.SetWaypoint(wp1L);
    fp.SetWaypoint(wp1M);
    fp.SetWaypoint(wp1H);
    fp.SetWaypoint(wp3);
    fp.SetWaypoint(wp2H);
    fp.SetWaypoint(wp2M);
    fp.SetWaypoint(wp2L);
    
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
        if strcmp(obj.UAVs(i).id, id)
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

    % time = double(msg.time.sec) + double(msg.time.nanosec)/1E9;
    % disp("NavigationReport received")

    if msg.operator_id ~= obj.name
        fprintf("WARNING: Operator '%s' has received a report from a not operated UAV %s \n\n",obj.name,msg.uav_id);
        return
    end

    if msg.fp_aborted
        fprintf("%s has discarded the proposed flight plan \n",msg.uav_id);
    end

    % if msg.fp_running
    %     if (msg.current_wp == 0)
    %         fprintf("%s waiting at starting WP0 \n",msg.uav_id);
    %     elseif (msg.current_wp == 1)
    %         fprintf("%s starting flight plan \n",msg.uav_id);
    %     else
    %         fprintf("%s heading WP%d \n",msg.uav_id,msg.current_wp);
    %     end
    % end

    if msg.fp_completed

        % fprintf("%s has completed its flight plan \n",msg.uav_id);

        if msg.plan_id == 0
            return;
        end
        
        UAVloc = obj.GetUAVLocation(msg.uav_id);
        op = obj.ops(msg.plan_id);
        if msg.uav_id ~= op.UAVid
            fprintf("WARNING: Drone %s is executing a corrupted flight plan \n\n",msg.uav_id);
            return;
        end

        if strcmp(UAVloc,op.VPdest)
            % fprintf("%s has reached its destination vertiport \n",msg.uav_id);
            obj.OperateUAV(op.UAVid);
        else
            fprintf("\n%s has not reached its destination vertiport \n",msg.uav_id);
            % disp(msg.uav_id)
            % disp(UAVloc)
            % disp(op.VPdest)
        end
    end
end



end % methods 
end % classdef

