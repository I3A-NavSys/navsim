classdef SimpleMonitor < handle

properties
    name   char               % Monitor name
    UAVs = struct([])         % list of UAVs being tracked

    % ROS2 interface
    rosNode                   % ROS2 Node 
    rosSer_TrackUAV           % ROS2 Service server to accept tracking request
    
end

methods


function obj = SimpleMonitor(name)

    obj.name = char(name);

    % ROS2 node
    obj.rosNode = ros2node(obj.name);

    obj.rosSer_TrackUAV = ros2svcserver(obj.rosNode, ...
        '/NavSim/TrackUAV','navsim_msgs/TrackUAV', ...
        @obj.TrackUAVCallback);
    
end



function TrackUAV(obj,UAVid)

    if obj.GetUAVindex(UAVid) ~= -1
        return
    end
    
    uav.id = char(UAVid);
    uav.data = double.empty(0,7);
    uav.rosSub_Telemetry = ros2subscriber(obj.rosNode, ...
        ['/NavSim/' uav.id '/Telemetry'],'navsim_msgs/Telemetry', ...
        @obj.TelemetryCallback);

    obj.UAVs = [obj.UAVs uav];
end



function [errorMed,errorMax,timeMax] = PathFollowingError(obj,UAVid,fp)

    uav = obj.GetUAVindex(UAVid);
    if uav == -1
        return
    end

    UAVdata = obj.UAVs(uav).data;
    errorMed   = 0;
    errorMax   = 0;
    numSamples = 0;

    for i = 1:size(UAVdata,1)
        t = UAVdata(i,1);
        if t < fp.InitTime
            continue
        end
        if t > fp.FinishTime
            break
        end
        posUAV = UAVdata(i,2:4);
        wp = fp.StatusAtTime(t);
        posFP = wp.pos;
        posError = norm(posFP - posUAV);
        if posError > errorMax
           errorMax = posError;
           timeMax  = t;
        end
        errorMed = errorMed + posError;
        numSamples = numSamples +1;
    end
    errorMed = errorMed / numSamples;

end



function PositionFigure(obj,UAVid,fp,time_step)

    i = obj.GetUAVindex(UAVid);
    if i == -1
        return
    end

    % UAV data
    UAVdata = obj.UAVs(i).data;
    if isempty(UAVdata)
        return
    end

    FPdata  = fp.Trace(time_step);

    % Checking figure
    figName = [UAVid ' position'];
    fig = findobj('Type','figure','Name',figName)';
    if (isempty(fig)) 
        fig = figure("Name", figName);
        fig.Position(3:4) = [800 400];
        fig.NumberTitle = "off";
    else
        figure(fig)
        clf(fig)
    end


    %Figure settings
    tl = tiledlayout(4,6);
    tl.Padding = 'compact';
    tl.TileSpacing = 'tight';
    color = [0 0.7 1];
    

    %Display Position 3D
    XYZtile = nexttile([4,4]);
    title("Position 3D")
    hold on
    grid on
    axis equal
    view(-20,20)
    xlabel("x [m]")
    ylabel("y [m]")
    zlabel("z [m]")
    
    plot3(FPdata(:,2), FPdata(:,3), FPdata(:,4), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    for vertice = fp.waypoints
        plot3(vertice.pos(1), vertice.pos(2), vertice.pos(3), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end   
    
    plot3(UAVdata(:,2), UAVdata(:,3), UAVdata(:,4), ...
        '.:', ...
        Color = 'black', ...
        MarkerSize = 8)


    %Display error in Position 3D
    XYZtile = nexttile([1,2]);
    title("Error 3D")
    hold on
    grid on

    xlabel("t [s]");
    ylabel("error [m]");

    plot([fp.InitTime fp.FinishTime], [fp.radius fp.radius], "--", Color = 'red' )
    % plot([fp.InitTime fp.FinishTime], [25 25], ".", Color = 'white' )

    timeValues  = [];
    errorValues = [];
    for i = 1:size(UAVdata,1)
        t = UAVdata(i,1);
        if t < fp.InitTime
            continue
        end
        if t > fp.FinishTime
            break
        end
        timeValues = [timeValues t];

        posUAV = UAVdata(i,2:4);
        wpFP  = fp.StatusAtTime(t);
        posError = norm(wpFP.pos - posUAV);
        errorValues = [errorValues posError];
    end

    plot(timeValues, errorValues, ...
        "-", ...
        LineWidth = 1, ...
        Color = 'black' )
    xlim([fp.InitTime fp.FinishTime])
    ax = gca; 
    ax.XAxis.Visible = 'off';
    
    disp(['Position error area (m^2) for ', UAVid]);
    disp(trapz(timeValues, errorValues));

    disp(['Flight time (s) for ', UAVid]);
    disp(fp.FinishTime - fp.InitTime);

    %Display Position X versus time
    Xtile   = nexttile([1,2]);
    title("Position versus time");
    hold on
    grid on
    ylabel("x [m]");

    plot(FPdata(:,1), FPdata(:,2), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )

    for vertice = fp.waypoints
        plot(vertice.t, vertice.pos(1), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end

    plot(UAVdata(:,1), UAVdata(:,2), ...
        ".:", ...
        Color = 'black', ...
        MarkerSize = 8 )
    xlim([fp.InitTime fp.FinishTime])
    ax = gca; 
    ax.XAxis.Visible = 'off';
        
    %Display Position Y versus time
    Ytile   = nexttile([1,2]);
    hold on
    grid on
    ylabel("y [m]");

    plot(FPdata(:,1), FPdata(:,3), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    for vertice = fp.waypoints
        plot(vertice.t, vertice.pos(2), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end    
    plot(UAVdata(:,1), UAVdata(:,3), ...
        ".:", ...
        Color = 'black', ...
        MarkerSize = 8)
    xlim([fp.InitTime fp.FinishTime])
    ax = gca; 
    ax.XAxis.Visible = 'off';


    %Display Position Z versus time
    Ztile = nexttile([1,2]);
    hold on
    grid on
    xlabel("t [s]");
    ylabel("z [m]");

    plot(FPdata(:,1), FPdata(:,4), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    for vertice = fp.waypoints
        plot(vertice.t, vertice.pos(3), ...
            'o',...
            MarkerSize = 5, ...
            MarkerFaceColor = 'white', ...
            MarkerEdgeColor = color )
    end

    plot(UAVdata(:,1), UAVdata(:,4), ...
        ".:", ...
        Color = 'black', ...
        MarkerSize = 8)
    xlim([fp.InitTime fp.FinishTime])

end



function VelocityFigure(obj,UAVid,fp,time_step)

    i = obj.GetUAVindex(UAVid);
    if i == -1
        return
    end

    % UAV data
    UAVdata = obj.UAVs(i).data;
    FPdata  = fp.Trace(time_step);
    

    % Checking figure
    figName = [UAVid ' velocity'];
    fig = findobj('Type','figure','Name',figName)';
    if (isempty(fig)) 
        fig = figure("Name", figName);
        fig.Position(3:4) = [800 400];
        fig.NumberTitle = "off";
    else
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
    title("Velocity versus time")
    hold on
    grid on
    ylabel("3D [m/s]");

    plot(FPdata(:,1),sqrt(FPdata(:,5).^2 + FPdata(:,6).^2 + FPdata(:,7).^2), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )

    plot(UAVdata(:,1),sqrt(UAVdata(:,5).^2 + UAVdata(:,6).^2 + UAVdata(:,7).^2), ...
        '.:', ...
        Color = 'black', ...
        MarkerSize = 8)
    xlim([fp.InitTime fp.FinishTime])


    %Display Velocity X versus time
    Xtile   = nexttile([1,2]);
    hold on
    grid on
    ylabel("vx [m/s]");

    plot(FPdata(:,1),FPdata(:,5), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    plot(UAVdata(:,1),UAVdata(:,5), ...
        '.:', ...
        Color = 'black', ...
        MarkerSize = 8)

    xlim([fp.InitTime fp.FinishTime])
    ax = gca; 
    ax.XAxis.Visible = 'off';
        
    %Display Position Y versus time
    Ytile   = nexttile([1,2]);
    hold on
    grid on
    ylabel("xy [m/s]");

    plot(FPdata(:,1),FPdata(:,6), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    plot(UAVdata(:,1),UAVdata(:,6), ...
        '.:', ...
        Color = 'black', ...
        MarkerSize = 8)

    xlim([fp.InitTime fp.FinishTime])
    ax = gca; 
    ax.XAxis.Visible = 'off';



    %Display Position Z versus time
    Ztile   = nexttile([1,2]);
    hold on
    grid on
    xlabel("t [s]");
    ylabel("vz [m/s]");

    plot(FPdata(:,1),FPdata(:,7), ...
        '-', ...
        LineWidth = 2, ...
        Color = color )
    plot(UAVdata(:,1),UAVdata(:,7), ...
        '.:', ...
        Color = 'black', ...
        MarkerSize = 8)

    xlim([fp.InitTime fp.FinishTime])

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

    time = double(msg.time.sec) + double(msg.time.nanosec)/1E9;
    pos = [msg.pose.position.x   msg.pose.position.y   msg.pose.position.z];
    vel = [msg.velocity.linear.x msg.velocity.linear.y msg.velocity.linear.z];

    i = obj.GetUAVindex(msg.uav_id);
    if i == -1
        return
    end

    if  ~isempty(obj.UAVs(i).data)
        if  (time < obj.UAVs(i).data(end,1))
            obj.UAVs(i).data = double.empty(0,7);
        end
    end
    obj.UAVs(i).data(end+1,:) = [time pos vel];

end



function TrackUAVCallback(obj,msg)

    
end


end % methods
end % classdef