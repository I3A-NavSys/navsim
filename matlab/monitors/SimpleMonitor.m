classdef SimpleMonitor < handle

properties
    name    string            % Monitor name
    UAVs = struct([])         % list of UAVs being tracked

    % ROS2 interface
    rosNode                   % ROS2 Node 
    rosSer_TrackUAV           % ROS2 Service server to accept tracking request
    
end

methods


function obj = SimpleMonitor(name)

    obj.name = name;

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

    uav.id = UAVid;
    uav.data = double.empty(0,7);
    uav.rosSub_Telemetry = ros2subscriber(obj.rosNode, ...
        ['/NavSim/' UAVid '/Telemetry'],'navsim_msgs/Telemetry', ...
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
        posFP  = fp.PositionAtTime(t);
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



function PositionFigure(obj,UAVid,fp)

    i = obj.GetUAVindex(UAVid);
    if i == -1
        return
    end

    % UAV data
    UAVdata = obj.UAVs(i).data;
    timeStep = UAVdata(2,1) - UAVdata(1,1);
    FPdata  = fp.Trace(timeStep);
    

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
    plot3([fp.waypoints(:).x], [fp.waypoints(:).y], [fp.waypoints(:).z], ...
        'o', ...
        MarkerSize = 5, ...
        MarkerFaceColor = 'white', ...
        MarkerEdgeColor = color )

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
        posFP  = fp.PositionAtTime(t);
        posError = norm(posFP - posUAV);
        errorValues = [errorValues posError];
    end

    plot(timeValues, errorValues, ...
        "-", ...
        LineWidth = 2, ...
        Color = 'red' )
    xlim([fp.InitTime fp.FinishTime])
    ax = gca; 
    ax.XAxis.Visible = 'off';



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
    plot([fp.waypoints(:).t], [fp.waypoints(:).x], ...
        'o',...
        MarkerSize = 5, ...
        MarkerFaceColor = 'white', ...
        MarkerEdgeColor = color )

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
    plot([fp.waypoints(:).t], [fp.waypoints(:).y], ...
        'o',...
        MarkerSize = 5, ...
        MarkerFaceColor = 'white', ...
        MarkerEdgeColor = color )

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
    plot([fp.waypoints(:).t], [fp.waypoints(:).z], ...
        'o',...
        MarkerSize = 5, ...
        MarkerFaceColor = 'white', ...
        MarkerEdgeColor = color )

    plot(UAVdata(:,1), UAVdata(:,4), ...
        ".:", ...
        Color = 'black', ...
        MarkerSize = 8)
    xlim([fp.InitTime fp.FinishTime])

end



function VelocityFigure(obj,UAVid,fp)

    i = obj.GetUAVindex(UAVid);
    if i == -1
        return
    end

    % UAV data
    UAVdata = obj.UAVs(i).data;
    timeStep = UAVdata(2,1) - UAVdata(1,1);
    FPdata  = fp.Trace(timeStep);
    

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