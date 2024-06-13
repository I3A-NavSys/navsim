classdef FlightPlan < handle
%FLIGHTPLAN Summary of this class goes here
%   Detailed explanation goes here

properties

id          {mustBeNumeric}    % operation identifier
waypoints   Waypoint = Waypoint.empty;     
radius      {mustBeNumeric}    % safety bubble for conflict management
priority    int8;

end


methods

%-------------------------------------------------------------------
% ROUTE COMPOSITION


    
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
end



function SetWaypoint(obj,waypoint)
    
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



function AppendWaypoint(obj,waypoint)
% introduces a WP at the end (1s later) of the flight plan

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



function fp2 = Copy(fp1)
    % Dado un plan, genera una copia independiente 
    fp2 = FlightPlan(Waypoint.empty);
    fp2.radius   = fp1.radius;
    fp2.priority = fp1.priority;

    for i = 1 : length(fp1.waypoints)
        wp1 = fp1.waypoints(i);
        wp2 = Waypoint();
        wp2.t = wp1.t;
        wp2.label = wp1.label;
        wp2.pos  = wp1.pos;
        wp2.vel  = wp1.vel;
        wp2.acel = wp1.acel;
        wp2.jerk = wp1.jerk;
        wp2.jolt = wp1.jolt;
        wp2.snap = wp1.snap;
        wp2.mandatory = wp1.mandatory;
        fp2.SetWaypoint(wp2);
    end
end



%-------------------------------------------------------------------
% TIME MANAGEMENT



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



function SetTimeFromVel(obj, label, vel)
    i = obj.GetIndexFromLabel(label);
    if i <= 1
        return
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);

    % wp1.Stop;
    % wp1.vel = vel * wp1.DirectionTo(wp2);
    t = wp1.t + wp1.DistanceTo(wp2) / vel;
    obj.PostponeFrom(wp2.t, t-wp2.t);
end



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
    if i > 1
        if timeStep < obj.waypoints(i).TimeTo(obj.waypoints(i-1))
            return      % there is not time enough in the past
        end
    end
    for j = i:length(obj.waypoints)
        obj.waypoints(j).Postpone(timeStep);
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




%-------------------------------------------------------------------
% FLIGHT PLAN BEHAVIOUR



function wp2 = StatusAtTime(obj, t)

    % Check if t is out of flight plan schedule, returning not valid pos
    if t < obj.InitTime  ||  t > obj.FinishTime
        wp2 = Waypoint;
        wp2.pos = [NaN NaN NaN];
        return
    end

    % search for the current waypoints
    for i = 2:length(obj.waypoints)
        if t < obj.waypoints(i).t
            break
        end
    end

    wp1 = obj.waypoints(i-1);
    wp2 = wp1.Interpolation(t);

end



function tr = Trace(obj, timeStep)
    % This method expands the flight plan behavior over time
    
    instants = obj.InitTime : timeStep : obj.FinishTime;
    tr = zeros(length(instants),7);
    tr(:,1) = instants;
   
    %Get position in time instants
    for i = 1:length(instants)
        wp = obj.StatusAtTime(tr(i,1));
        tr(i,2:4) = wp.pos;
        tr(i,5:7) = wp.vel;
    end
    tr(end,5:7) = [0 0 0];
end



%-------------------------------------------------------------------
% ROUTE MANAGEMENT



function SetV0000(obj)
    % Para cada waypoint asigna su vector velocidad 
    % asumiendo movimiento rectilíneo y uniforme

    for i = 1 : length(obj.waypoints)-1
        wpA = obj.waypoints(i);
        wpB = obj.waypoints(i+1);
        wpA.SetV0000(wpB);
    end
    wpB.Stop();
end



function SetJLS(obj)
    % Para cada waypoint con
    % tiempo, posición, velocidad y aceleración determinados
    % obtiene las 3 derivadas siguientes que ejecutan dicho movimiento

    for i = 1 : length(obj.waypoints)-1
        wpA = obj.waypoints(i);
        wpB = obj.waypoints(i+1);
        wpA.SetJLS(wpB);
    end
end



function SmoothVertexMaintainingSpeed(obj,label,ang_vel)
    % Curva el vertice entre dos rectas
    % manteniendo velocidad y acortando el tiempo de vuelo.
    % Para ello descompone dicho waypoint en dos.

    i = obj.GetIndexFromLabel(label);
    if i <= 1  ||  length(obj.waypoints) <= i
        return
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);    
    if wp2.mandatory
        return
    end

    angle = wp1.AngleWith(wp2);

    v1  = norm(wp1.vel);
    v2  = norm(wp2.vel);
    v   = mean([v1 v2]);
    
    r = v / ang_vel;         % radius of the curve
    d = r * tan(angle/2);    % distance to the new waypoints
    step = d / v;            % time to the decomposed waypoints

    wp2A = Waypoint;
    wp2A.label = strcat(wp2.label,'_A');
    wp2A.pos = wp2.pos - wp1.vel * step;
    wp2A.vel = wp1.vel;
    wp2A.t = wp2.t - step;

    wp2B = Waypoint;
    wp2B.label = strcat(wp2.label,'_B');
    wp2B.pos = wp2.pos + wp2.vel * step;
    wp2B.vel = wp2.vel;
    wp2B_t_init = wp2.t + step;
   
    t2_min = wp2A.t + angle/ang_vel;
    t2_max = wp2B_t_init;

    while t2_max-t2_min > 0.05
        % [t2_min t2_max]

        wp2B.t = mean([t2_min t2_max])
        wp2A.SetJLS(wp2B);
        
        tAB_med = mean([wp2A.t wp2B.t]);
        status = wp2A.Interpolation(tAB_med);
        v_med = norm(status.vel);
        if v_med < v
            t2_max = wp2B.t;
        else
            t2_min = wp2B.t;
        end
    end


    obj.RemoveWaypointAtTime(wp2.t);
    obj.SetWaypoint(wp2A);
    obj.SetWaypoint(wp2B);

    obj.PostponeFrom(wp2B.t + 0.001, wp2B.t-wp2B_t_init);

end




function SmoothVertexMaintainingDuration(obj,label,ang_vel,lin_acel)
    % Curva el vertice entre dos rectas
    % reduciendo velocidad y manteniendo el tiempo de vuelo.
    % Para ello descompone dicho waypoint en dos.

    i = obj.GetIndexFromLabel(label);
    if i <= 1  ||  length(obj.waypoints) <= i
        return
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);    
    if wp2.mandatory
        return
    end

    angle = wp1.AngleWith(wp2);
    tc = angle / ang_vel;     % time spent in the curve

    v1  = norm(wp1.vel);
    v2  = norm(wp2.vel);
    ts  = abs(v2-v1)/lin_acel;

    interval = max([tc ts]);
    obj.ExpandWaypoint(label,interval)
        
end



function ExpandWaypoint(obj,label,interval)
% descompone un waypoint en dos, separados un intervalo dado

    i = obj.GetIndexFromLabel(label);
    if i <= 1  ||  length(obj.waypoints) <= i
        return
    end

    wp1 = obj.waypoints(i-1);
    wp2 = obj.waypoints(i);    
    wp3 = obj.waypoints(i+1);

    step = interval/2;
    if step >= wp2.t - wp1.t || step >= wp3.t - wp2.t
       return  % there is not time enough to include the curve
    end

    wp2A = Waypoint;
    wp2A.label = strcat(wp2.label,'_A');
    wp2A.pos = wp2.pos - wp1.vel * step;
    wp2A.vel = wp1.vel;
    wp2A.t = wp2.t - step;

    wp2B = Waypoint;
    wp2B.label = strcat(wp2.label,'_B');
    wp2B.pos = wp2.pos + wp2.vel * step;
    wp2B.vel = wp2.vel;
    wp2B.t = wp2.t + step;

    wp2A.SetJLS(wp2B);

    obj.RemoveWaypointAtTime(wp2.t);
    obj.SetWaypoint(wp2A);
    obj.SetWaypoint(wp2B);

end



%-------------------------------------------------------------------
% CONFLICT DETECTION



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
        wp1 = fp1.StatusAtTime(t);
        wp2 = fp2.StatusAtTime(t);
        dist(i,2) = norm(wp2.pos-wp1.pos);
    end
end



%-------------------------------------------------------------------
% FIGURES



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
    nexttile([3,3]);
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
    for vertice = obj.waypoints
        plot3(vertice.pos(1), vertice.pos(2), vertice.pos(3), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end   
    %Display Position X versus time
    nexttile([1,2]);
    title("Position versus time");
    hold on
    grid on
    % xlabel("t [s]");
    ylabel("x [m]");

    plot(tr(:,1),tr(:,2), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    for vertice = obj.waypoints
        plot(vertice.t, vertice.pos(1), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'w', ...
            MarkerEdgeColor = color )
    end
    %Display Position Y versus time
    nexttile([1,2]);
    % title("Position Y");
    hold on
    grid on
    % xlabel("t [s]");
    ylabel("y [m]");

    plot(tr(:,1),tr(:,3), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    for vertice = obj.waypoints
        plot(vertice.t, vertice.pos(2), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'w', ...
            MarkerEdgeColor = color )
    end

    %Display Position Z versus time
    nexttile([1,2]);
    % title("Position Z");
    hold on
    grid on
    xlabel("t [s]");
    ylabel("z [m]");

    plot(tr(:,1),tr(:,4), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    for vertice = obj.waypoints
        plot(vertice.t, vertice.pos(3), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'w', ...
            MarkerEdgeColor = color )
    end

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
    nexttile([1,2]);
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
    nexttile([1,2]);
    hold on
    grid on
    ylabel("vx [m/s]");

    plot(tr(:,1),tr(:,5), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )

    for vertice = obj.waypoints
        plot(vertice.t, vertice.vel(1), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end

    %Display Position Y versus time
    nexttile([1,2]);
    hold on
    grid on
    ylabel("vy [m/s]");

    plot(tr(:,1),tr(:,6), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )

    for vertice = obj.waypoints
        plot(vertice.t, vertice.vel(2), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end

    %Display Position Z versus time
    nexttile([1,2]);
    hold on
    grid on
    ylabel("vz [m/s]");
    xlabel("t [s]");

    plot(tr(:,1),tr(:,7), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )


    for vertice = obj.waypoints
        plot(vertice.t, vertice.vel(3), ...
            'o',...
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


