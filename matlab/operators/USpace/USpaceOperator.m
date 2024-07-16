
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
        
        
        
        v2 = 3;
        
        
        
        
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
    
    % Create waypoints

    %take off positions
    wpA1 = Waypoint();      
    wpA1.label = 'wpA1';
    wpA1.pos = vp1 + [0 0 0.25];

    wpA2 = Waypoint();
    wpA2.label = 'wpA2';
    wpA2.pos = wpA1.pos;


    % landing position
    wpB1 = Waypoint();      
    wpB1.label = 'wpB1';
    wpB1.pos = vp2 + [0 0 0.25];

    wpB2 = Waypoint();      
    wpB2.label = 'wpB2';
    wpB2.pos = wpB1.pos;


    %intermediate positions
    angle = wpA1.CourseTo(wpB1);
    altitude = 70 + angle/20;

    wpA3 = Waypoint();      
    wpA3.label = 'wpA3';
    wpA3.pos = wpA1.pos;
    wpA3.pos(3) = altitude - 10;

    wpB3 = Waypoint();      
    wpB3.label = 'wpB3';
    wpB3.pos = wpB1.pos;
    wpB3.pos(3) = altitude - 10;


    wpA4 = Waypoint();      
    wpA4.label = 'wpA4';
    wpA4.pos = wpA1.pos;
    wpA4.pos(3) = altitude;

    wpB4 = Waypoint();      
    wpB4.label = 'wpB4';
    wpB4.pos = wpB1.pos;
    wpB4.pos(3) = altitude;
    
    wpA5 = Waypoint();      
    wpA5.label = 'wpA5';
    wpA5.pos = wpA4.pos + 10 * wpA4.DirectionTo(wpB4);

    wpB5 = Waypoint();      
    wpB5.label = 'wpB5';
    wpB5.pos = wpB4.pos + 10 * wpB4.DirectionTo(wpA4);


    % compose the flight plan
    fp = FlightPlan(Waypoint.empty);
    fp.radius = 2;
    fp.AppendWaypoint(wpA1);
    wpA2.t = fp.FinishTime + 5;
    fp.SetWaypoint(wpA2);
    fp.AppendWaypoint(wpA3);
    fp.AppendWaypoint(wpA4);
    fp.AppendWaypoint(wpA5);
    fp.AppendWaypoint(wpB5);
    fp.AppendWaypoint(wpB4);
    fp.AppendWaypoint(wpB3);
    fp.AppendWaypoint(wpB2);
    wpB1.t = fp.FinishTime + 5;
    fp.SetWaypoint(wpB1);
    
    % waypoint time intervals
    % establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
    fp.SetTimeFromVel('wpA3', 2);
    fp.SetTimeFromVel('wpA4', 2);
    fp.SetTimeFromVel('wpA5', 2);
    fp.SetTimeFromVel('wpB5', 8);
    fp.SetTimeFromVel('wpB4', 2);
    fp.SetTimeFromVel('wpB3', 2);
    fp.SetTimeFromVel('wpB2', 2);

    fp.SetV0000;
    % fp.Print;
    % fp.PositionFigure("FP: POSITION",0.01);
    % fp.VelocityFigure("FP: VELOCITY",0.01);

    ang_vel  = 1.0;
    lin_acel = 1.0;

    fp.SmoothVertexMaintainingDuration('wpA2',ang_vel,lin_acel);
    fp.SmoothVertexMaintainingDuration('wpB2',ang_vel,lin_acel);
    % fp.Print;
    % fp.PositionFigure("FP: POSITION",0.01);
    % fp.VelocityFigure("FP: VELOCITY",0.01);
      
    fp.SmoothVertexMaintainingSpeed('wpA4',ang_vel);
    fp.SmoothVertexMaintainingSpeed('wpB4',ang_vel);
    % fp.Print;
    % fp.PositionFigure("FP: POSITION",0.01);
    % fp.VelocityFigure("FP: VELOCITY",0.01);

    fp.SmoothVertexMaintainingDuration('wpA5',ang_vel,lin_acel);
    fp.SmoothVertexMaintainingDuration('wpB5',ang_vel,lin_acel);
    % fp.Print;
    % fp.PositionFigure("FP: POSITION",0.01);
    % fp.VelocityFigure("FP: VELOCITY",0.01);

    
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
        % if fp.mode == InterpolationModes.TP
        % else
        %     return
        % end
    else
        return
    end

    % Send ROS2 message
    msg = ros2message(uav.rosPub_FlightPlan);
    msg.plan_id     = uint16(fp.id);
    msg.uav_id      = char(uav.id);
    msg.operator_id = char(obj.name);
    msg.priority    = int8(fp.priority);
    % msg.mode        = char(fp.mode);
    msg.radius      = fp.radius;

    for i = 1:length(fp.waypoints)
        t = fp.waypoints(i).t;
        msg.route(i).time.sec = int32(floor(t));
        msg.route(i).time.nanosec = uint32(rem(t,1)*1E9);

        msg.route(i).pos.x = fp.waypoints(i).pos(1);
        msg.route(i).pos.y = fp.waypoints(i).pos(2);
        msg.route(i).pos.z = fp.waypoints(i).pos(3);

        msg.route(i).vel.x = fp.waypoints(i).vel(1);
        msg.route(i).vel.y = fp.waypoints(i).vel(2);
        msg.route(i).vel.z = fp.waypoints(i).vel(3);

        msg.route(i).acel.x = fp.waypoints(i).acel(1);
        msg.route(i).acel.y = fp.waypoints(i).acel(2);
        msg.route(i).acel.z = fp.waypoints(i).acel(3);

        msg.route(i).jerk.x = fp.waypoints(i).jerk(1);
        msg.route(i).jerk.y = fp.waypoints(i).jerk(2);
        msg.route(i).jerk.z = fp.waypoints(i).jerk(3);

        msg.route(i).jolt.x = fp.waypoints(i).jolt(1);
        msg.route(i).jolt.y = fp.waypoints(i).jolt(2);
        msg.route(i).jolt.z = fp.waypoints(i).jolt(3);

        msg.route(i).snap.x = fp.waypoints(i).snap(1);
        msg.route(i).snap.y = fp.waypoints(i).snap(2);
        msg.route(i).snap.z = fp.waypoints(i).snap(3);

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

