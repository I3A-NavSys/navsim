classdef Path5D < handle
    
properties

    waypoints Pose5D    % secuencia de puntos de referencia en la ruta

end %properties

methods


function setWaypoint(obj,t,x,y,z,v)
    pose = Pose5D;
    pose = pose.setPose5D(t,x,y,z,v);

    l = length(obj.waypoints);
    for i = 1:l
        if obj.waypoints(i).t == pose.t
            obj.waypoints(i) = pose;
            return
        end
        if obj.waypoints(i).t > pose.t
            for j = l:-1:i
                obj.waypoints(j+1) = obj.waypoints(j);
            end
            obj.waypoints(i) = pose;
            return
        end
    end
    obj.waypoints(l+1) = pose;
end


function [t_init,t_end] = getInterval(obj)
    t_init = -1;
    t_end  = -1;

    if isempty(obj.waypoints)
        return
    else
        t_init = obj.waypoints(1).t;
        t_end  = obj.waypoints(end).t;
    end
end


function p = getPose4DAtTime(obj,t)
    [t_init,t_end] = obj.getInterval;
    if t_init==-1 || t < t_init || t > t_end
        p = Pose5D;
        return
    end
    l = length(obj.waypoints);
    for i = 1:l
        p1 = obj.waypoints(i);
        if p1.t == t
            p = p1;
            return
        elseif i<l 
            p2 = obj.waypoints(i+1);
            if p1.t < t && t < p2.t
                %% interpolamos entre p1 y p2
                p = p1.interpolation4D(p2,t);
                return
            end
        end
    end
end


function p = getPose5DAtTime(obj,t)
    [t_init,t_end] = obj.getInterval;
    if t_init==-1 || t < t_init || t > t_end
        p = Pose5D;
        return
    end
    l = length(obj.waypoints);
    for i = 1:l
        p1 = obj.waypoints(i);
        if p1.t == t
            p = p1;
            return
        elseif i<l 
            p2 = obj.waypoints(i+1);
            if p1.t < t && t < p2.t
                %% interpolamos entre p1 y p2
                p = p1.interpolation5D(p2,t);
                return
            end
        end
    end
end



function showPoses4D(obj,name,step)

    figHandler = findobj('Type','figure','Name',name)';
    if (isempty(figHandler)) 
        figure('Name',name, ...
               'NumberTitle','off');
    else
        figure(figHandler)
        clf
    end
    
    [t1,t2] = obj.getInterval;
    XYZ = zeros(length(t1:step:t2),3);
    i = 1;
    for t = t1 : step : t2
        p = obj.getPose4DAtTime(t);
        XYZ(i,1) = p.x;
        XYZ(i,2) = p.y;
        XYZ(i,3) = p.z;
        i = i+1;
    end
    plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'+-', ...
        LineWidth=2,Color='b')
    grid on
end


function showVels4D(obj,name,step)

    figHandler = findobj('Type','figure','Name',name)';
    if (isempty(figHandler)) 
        figure('Name',name, ...
               'NumberTitle','off');
    else
        figure(figHandler)
        clf
    end

    [t1,t2] = obj.getInterval;
    v = zeros(length(t1:step:t2),1);
    i = 1;
    for t = t1 : step : t2
        p1 = obj.getPose4DAtTime(t);
        p2 = obj.getPose4DAtTime(t+step);
        v(i) = p1.distance(p2) / step;
        i = i+1;
    end
    plot(t1:step:t2, v, ...
        LineWidth=2,Color='r')
    grid on

end


function showPoses5D(obj,name,step)

    figHandler = findobj('Type','figure','Name',name)';
    if (isempty(figHandler)) 
        figure('Name',name, ...
               'NumberTitle','off');
    else
        figure(figHandler)
        clf
    end
    
    [t1,t2] = obj.getInterval;
    XYZ = zeros(length(t1:step:t2),3);
    i = 1;
    for t = t1 : step : t2
        p = obj.getPose5DAtTime(t);
        XYZ(i,1) = p.x;
        XYZ(i,2) = p.y;
        XYZ(i,3) = p.z;
        i = i+1;
    end
    plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'+-', ...
        LineWidth=2,Color='b')
    grid on
end


function showVels5D(obj,name,step)

    figHandler = findobj('Type','figure','Name',name)';
    if (isempty(figHandler)) 
        figure('Name',name, ...
               'NumberTitle','off');
    else
        figure(figHandler)
        clf
    end

    [t1,t2] = obj.getInterval;
    v = zeros(length(t1:step:t2),1);
    i = 1;
    for t = t1 : step : t2
        p1 = obj.getPose5DAtTime(t);
        p2 = obj.getPose5DAtTime(t+step);
        v(i) = p1.distance(p2) / step;
        i = i+1;
    end
    plot(t1:step:t2, v, ...
        LineWidth=2,Color='r')
    grid on
end


end %methods
end %class


