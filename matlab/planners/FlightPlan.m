classdef FlightPlan < handle
%FLIGHTPLAN Summary of this class goes here
%   Detailed explanation goes here

properties

id          {mustBeNumeric}    % operation identifier

waypoints   Waypoint = Waypoint.empty;     
mode        InterpolationModes;
radius      {mustBeNumeric}    % safety bubble for conflict management

priority    int8;

end


methods


    
function obj = FlightPlan(waypoints)
    obj.id = 0;
    obj.priority = 0;
    obj.radius   = 1;

    %Check if the waypoints list is empty ([])
    if ~isempty(waypoints)
        %Add waypoints to the flight plan
        for i = 1:length(waypoints)
            obj.SetWaypoint(waypoints(i));
        end
    end
    obj.mode = "TP";
end



function SetWaypoint(obj,waypoint)
    
    %Check if the waypoint is a Waypoint object
    if ~isa(waypoint,'Waypoint')
        error('The waypoint must be a Waypoint object');
    end

    l = length(obj.waypoints);
    for i = 1:l
        if obj.waypoints(i).t == waypoint.t
            obj.waypoints(i) = waypoint;
            return
        end
        if obj.waypoints(i).t > waypoint.t
            obj.waypoints = [obj.waypoints(1:i-1)  waypoint  obj.waypoints(i:l)];
            return
        end
    end
    obj.waypoints(l+1) = waypoint;

end



function InsertWaypoint(obj,waypoint)
% introduces a WP at the end (1s later) of the flight plan

    %Check if the waypoint is a Waypoint object
    if ~isa(waypoint,'Waypoint')
        error('The waypoint must be a Waypoint object');
    end

    if isempty(obj.waypoints)
        waypoint.t = 0;
    else
        waypoint.t = obj.FinishTime + 1;
    end

    obj.SetWaypoint(waypoint);

end



function RemoveWaypointAtTime(obj,t)

    l = length(obj.waypoints);
    for i = 1:l
        if obj.waypoints(i).t == t
            if l == 1
                obj.waypoints = Waypoint.empty;
            else
                obj.waypoints = [obj.waypoints(1:i-1) obj.waypoints(i+1:end)];
            end
            return
        end
    end

end



function t = InitTime(obj)
%time of the first waypoint
    if isempty(obj.waypoints)
        t = [];
    else
        t = obj.waypoints(1).t;
    end
end



function t = FinishTime(obj)
%time of the last waypoint
    if isempty(obj.waypoints)
        t = [];
    else
        t = obj.waypoints(end).t;
    end        
end



function i = GetIndexFromLabel(obj, label)
    i = 0;
    for j = 1:length(obj.waypoints)
        wp = obj.waypoints(j);
        if strcmp(label,wp.label)
            i = j;
            return
        end
    end    
end



% function SetTimeFromVel(obj, label, vel)
%     if obj.mode ~= InterpolationModes.TP
%         return
%     end
%     i = obj.GetIndexFromLabel(label);
%     if i <= 1
%         return
%     end
% 
%     wp1 = obj.waypoints(i-1);
%     wp2 = obj.waypoints(i);
%     t = wp1.t + wp1.DistanceTo(wp2) / vel;
%     obj.PostponeFrom(wp2.t, t-wp2.t);
% end



function PostponeFrom(obj, time, timeStep)
    % Postpone a portion of the Fligt Plan a given timeStep, 
    % starting from a given time
    
    if obj.FinishTime() < time 
        return
    end

    for i = 1:length(obj.waypoints)
        if time <= obj.waypoints(i).t 
            break
        end
    end

    for i = i:length(obj.waypoints)
        obj.waypoints(i).Postpone(timeStep);
    end
end



function Postpone(obj, timeStep)
    % Postpone the Fligt Plan a given timeStep 
    obj.PostponeFrom(obj.InitTime(),timeStep);
end



function RescheduleAt(obj, time)
    % Perform a temporal translation of the Fligt Plan to begin at a given time 
    obj.Postpone(time - obj.InitTime());
end



function p = PositionAtTime(obj, t)

    p = [NaN NaN NaN];

    % Check if t is out of flight plan schedule, returning not valid pos
    if t < obj.InitTime  ||  t > obj.FinishTime
        return
    end

    % search for the current waypoints
    for i = 2:length(obj.waypoints)
        if t < obj.waypoints(i).t
            break
        end
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);

    switch obj.mode
        % case InterpolationModes.PV
        case InterpolationModes.TP
            wp3 = wp1.InterpolationTP(wp2,t);
        case InterpolationModes.TPV0
            wp3 = wp1.InterpolationTPV0(wp2,t);
        otherwise
            % warning('Unknown interpolation mode')
            return
    end
    p = wp3.Position;

end



function v = VelocityAtTime(obj, t)

    v = [NaN NaN NaN];

    % Check if t is out of flight plan schedule, returning not valid pos
    if t < obj.InitTime  ||  t > obj.FinishTime
        return
    end

    % search for the current waypoints
    for i = 2:length(obj.waypoints)
        if t < obj.waypoints(i).t
            break
        end
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);

    switch obj.mode
        % case InterpolationModes.PV
        case InterpolationModes.TP
            wp3 = wp1.InterpolationTP(wp2,t);
        case InterpolationModes.TPV0
            wp3 = wp1.InterpolationTPV0(wp2,t);
        otherwise
            % warning('Unknown interpolation mode')
            return
    end
    v = wp3.Velocity;

end



function tr = Trace(obj, timeStep)
    % This method expands the flight plan behavior over time
    
    instants = obj.InitTime : timeStep : obj.FinishTime;
    tr = zeros(length(instants),7);
    tr(:,1) = instants;
   
    %Get position in time instants
    for i = 1:length(instants)
        p = obj.PositionAtTime(tr(i,1));
        tr(i,2:4) = p;
        v = obj.VelocityAtTime(tr(i,1));
        tr(i,5:7) = v;
    end
    tr(end,5:7) = [0 0 0];
end



function dist = DistanceTo(fp1, fp2, timeStep)
    % This method obstains relative distance between two flight plans over time
    
    initTime   = min(fp1.InitTime,  fp2.InitTime);
    finishTime = max(fp1.FinishTime,fp2.FinishTime);
    
    instants = initTime : timeStep : finishTime;
    dist = zeros(length(instants),2);
    dist(:,1) = instants;
   
    %Get position in time instants
    for i = 1:length(instants)
        t = dist(i,1);
        p1 = fp1.PositionAtTime(t);
        p2 = fp2.PositionAtTime(t);
        dist(i,2) = norm(p2-p1);
    end
end



function fp2 = Copy(fp1)
    % Dado un plan, genera una copia independiente 
    fp2 = FlightPlan(Waypoint.empty);
    fp2.mode = fp1.mode;
    fp2.radius   = fp1.radius;
    fp2.priority = fp1.priority;

    for i = 1 : length(fp1.waypoints)
        wp1 = fp1.waypoints(i);
        wp2 = Waypoint();
        wp2.t = wp1.t;
        wp2.label = wp1.label;
        wp2.SetPosition(wp1.Position);
        wp2.SetVelocity(wp1.Velocity);
        wp2.mandatory = wp1.mandatory;
        fp2.SetWaypoint(wp2);
    end
end



function SetUniformVelocities(obj)
    % Para cada waypoint asigna su vector velocidad 
    % asumiendo movimiento rectilíneo y uniforme

    for i = 1 : length(obj.waypoints)-1
        wpA = obj.waypoints(i);
        wpB = obj.waypoints(i+1);

        dir = wpA.DirectionTo(wpB);
        vel = wpA.UniformVelocityTo(wpB);
        wpA.SetVelocity(dir*vel);
    end
    wpB.SetVelocity([0 0 0]);
end



function fp2 = Convert2TP(fp1,timeStep)
    % Transform a TPV / TPV0 flight plan to a TP flight plan

    fp2 = FlightPlan(Waypoint.empty);
    if fp1.mode == InterpolationModes.TP
        return
    end
    
    fp2.radius   = fp1.radius;
    fp2.priority = fp1.priority;

    %Get position in time instants
    for i = fp1.InitTime : timeStep : fp1.FinishTime
        wp = Waypoint();
        wp.t = i;
        p = fp1.PositionAtTime(i);
        wp.SetPosition(p);
        fp2.SetWaypoint(wp);
    end
end



function SmoothVertex(obj,label,angvel)
% genera una curva en dicho vertice

    i = obj.GetIndexFromLabel(label);
    if i <= 1  ||  length(obj.waypoints) <= i
        return
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);    
    wp3 = obj.waypoints(i+1);

    v1  = norm(wp1.Velocity);
    v2  = norm(wp2.Velocity);

    r = (v1+v2)/2 / angvel;         % radius of the curve
    angle = wp1.AngleWith(wp2);
    d = r * tan(angle/2);           % distance to the new waypoints

    if d+1 > wp1.DistanceTo(wp2) || d+1 > wp2.DistanceTo(wp3)
       return  % there is not space to include the curve
    end

    wp2A = Waypoint;
    wp2A.label = strcat(wp2.label,'_A');
    wp2A.SetPosition(wp2.Position + d * wp2.DirectionTo(wp1));
    wp2A.SetVelocity(wp1.Velocity);
    wp2A.t = wp2.t - d/v2;
    obj.SetWaypoint(wp2A);

    wp2B = Waypoint;
    wp2B.label = strcat(wp2.label,'_B');
    wp2B.SetPosition(wp2.Position + d * wp2.DirectionTo(wp3));
    wp2B.SetVelocity(wp2.Velocity);
    wp2B.t = wp2.t + d/v2 ;
    obj.SetWaypoint(wp2B);

    obj.RemoveWaypointAtTime(wp2.t);

end



function ApplyConstantDubinsAt(obj,label,angvel)

    i = obj.GetIndexFromLabel(label);
    if i <= 1  ||  length(obj.waypoints) <= i
        return
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);    
    wp3 = obj.waypoints(i+1);

    v1  = norm(wp1.Velocity);
    v2  = norm(wp2.Velocity);
    v3  = norm(wp3.Velocity);

    r = v2 / angvel;                % radius of the curve
    angle = wp1.AngleWith(wp2);
    d = r * tan(angle/2);           % distance to the new waypoints
    s = r * angle;                  % curved distance covered

    if d > wp1.DistanceTo(wp2) || d > wp2.DistanceTo(wp3)
       return  % there is not space for dubins maneuvre
    end

    wp2A = Waypoint;
    wp2A.label = strcat(wp2.label,'_A');
    wp2A.SetPosition(wp2.Position + d * wp2.DirectionTo(wp1));
%    wp2A.SetVelocity(wp1.DirectionTo(wp2) * v2 );
    % wp2A.t = wp1.t + wp1.DistanceTo(wp2A) * 4 / (3*v1 + v2);
    wp2A.SetVelocity(wp1.velocity);
    wp2A.t = wp2.t - d/v2;
    obj.SetWaypoint(wp2A);

    wp2B = Waypoint;
    wp2B.label = strcat(wp2.label,'_B');
    wp2B.SetPosition(wp2.Position + d * wp2.DirectionTo(wp3));
    wp2B.SetVelocity(wp2.Velocity);
    wp2B.t = wp2A.t + s/v2 ;
    obj.SetWaypoint(wp2B);

    obj.RemoveWaypointAtTime(wp2.t);

    t3 = wp2B.t + wp2B.DistanceTo(wp3) * 4 / (3*v2 + v3);
    obj.PostponeFrom(wp3.t, t3-wp3.t);

end



function PositionFigure(obj,figName,time_step)
    % Display the flight plan trajectory
    
    %Check if the flight plan is empty
    if isempty(obj.waypoints)
        disp('The flight plan is empty');
        return
    end

    %Find if the figure is already open
    fig = findobj("Name", figName);
    if isempty(fig)
        %Display a figure with the flight plan
        fig = figure("Name", figName);
        fig.Position(3:4) = [800 350];
        fig.NumberTitle = "off";
        % fig.MenuBar = "none";
        % fig.ToolBar = "none";

    else
        %Select the figure
        figure(fig)
        clf(fig)
    end

    %Figure settings
    tl = tiledlayout(3,5);
    tl.Padding = 'compact';
    tl.TileSpacing = 'tight';
    color = [0 0.7 1];
 
    %Display Position 3D
    XYZtile = nexttile([3,3]);
    title("Position 3D")
    hold on
    grid on
    axis equal
    % view(30,10)
    xlabel("x [m]")
    ylabel("y [m]")
    zlabel("z [m]")

    tr = obj.Trace(time_step);
    plot3(tr(:,2),tr(:,3),tr(:,4), ...
        '-', ...
        LineWidth = 2, ...
        Color = color );
    plot3([obj.waypoints(:).x], [obj.waypoints(:).y], [obj.waypoints(:).z], ...
        'o',...
        MarkerSize = 5, ...
        MarkerFaceColor = 'white', ...
        MarkerEdgeColor = color )
       
    %Display Position X versus time
    Xtile   = nexttile([1,2]);
    title("Position versus time");
    hold on
    grid on
    % xlabel("t [s]");
    ylabel("x [m]");

    plot(tr(:,1),tr(:,2), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    plot([obj.waypoints(:).t], [obj.waypoints(:).x], ...
        'o',...
        MarkerSize = 5, ...
        MarkerFaceColor = 'w', ...
        MarkerEdgeColor = color )

    %Display Position Y versus time
    Ytile   = nexttile([1,2]);
    % title("Position Y");
    hold on
    grid on
    % xlabel("t [s]");
    ylabel("y [m]");

    plot(tr(:,1),tr(:,3), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    plot([obj.waypoints(:).t], [obj.waypoints(:).y], ...
        'o',...
        MarkerSize = 5, ...
        MarkerFaceColor = 'white', ...
        MarkerEdgeColor = color )

    %Display Position Z versus time
    Ztile   = nexttile([1,2]);
    % title("Position Z");
    hold on
    grid on
    xlabel("t [s]");
    ylabel("z [m]");

    plot(tr(:,1),tr(:,4), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    plot([obj.waypoints(:).t], [obj.waypoints(:).z], ...
        'o',...
        MarkerSize = 5, ...
        MarkerFaceColor = 'white', ...
        MarkerEdgeColor = color )

end



function VelocityFigure(obj,figName,time_step)
    % Display the flight plan instant velocity
    
    % Check if the flight plan is empty
    if isempty(obj.waypoints)
        disp('The flight plan is empty');
        return
    end

    %Find if the figure is already open
    fig = findobj('Type', 'Figure',"Name", figName);
    if isempty(fig)
        %Display a figure with the flight plan
        fig = figure("Name", figName);
        fig.Position(3:4) = [290 455];
        fig.NumberTitle = "off";
        % fig.MenuBar = "none";
        % fig.ToolBar = "none";
    else
        %Select the figure
        figure(fig)
        clf(fig)
    end

    %Figure settings
    tl = tiledlayout(4,2);
    tl.Padding = 'compact';
    tl.TileSpacing = 'tight';
    color = [0 0.7 1];

    %Display instant velocity
    Vtile = nexttile([1,2]);
    title("Velocity versus time")
    hold on
    grid on
    ylabel("3D [m/s]");

    tr = obj.Trace(time_step);
    plot(tr(:,1),sqrt(tr(:,5).^2 + tr(:,6).^2 + tr(:,7).^2), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )

    %Display Velocity X versus time
    Xtile   = nexttile([1,2]);
    hold on
    grid on
    ylabel("vx [m/s]");

    plot(tr(:,1),tr(:,5), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    if obj.mode == InterpolationModes.TPV0
        plot([obj.waypoints(:).t], [obj.waypoints(:).vx], 'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end

    %Display Position Y versus time
    Ytile   = nexttile([1,2]);
    hold on
    grid on
    ylabel("vy [m/s]");

    plot(tr(:,1),tr(:,6), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    if obj.mode == InterpolationModes.TPV0
        plot([obj.waypoints(:).t], [obj.waypoints(:).vy], 'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end

    %Display Position Z versus time
    Ztile   = nexttile([1,2]);
    hold on
    grid on
    ylabel("vz [m/s]");
    xlabel("t [s]");

    plot(tr(:,1),tr(:,7), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    if obj.mode == InterpolationModes.TPV0
        plot([obj.waypoints(:).t], [obj.waypoints(:).vz], 'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end
end



function DistanceFigure(fp1,fp2,time_step)
    % Relative distance between two flightplans
    
    %Find if the figure is already open
    fig_name = "FP" + fp1.id + "-FP" + fp2.id + ": DISTANCE";
    fig = findobj('Type', 'Figure',"Name", fig_name);
    if isempty(fig)
        fig = figure("Name", fig_name);
        fig.Position(3:4) = [350 140];
        fig.NumberTitle = "off";
        fig.MenuBar = "none";
        fig.ToolBar = "none";
    else
        figure(fig)
        clf(fig)
    end

    %Figure settings
    ylabel("distance [m]")
    xlabel("time [s]")
    grid on
    hold on
        
    dist = fp1.DistanceTo(fp2, time_step);
    plot(dist(:,1),dist(:,2), Color = 'green' );            
end


end % methods
end % classdef


