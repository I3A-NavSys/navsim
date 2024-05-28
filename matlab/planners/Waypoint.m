classdef Waypoint < handle

properties

label = '';             % label to refer the waypoint
t = 0    % time (s)

pos  = [0 0 0];    % position      (m)
vel  = [0 0 0];    % velocity      (m/s)
acel = [0 0 0];    % aceleration   (m/s2)
jerk = [0 0 0];    % jerk          (m/s3)
jolt = [0 0 0];    % jolt          (m/s4)
snap = [0 0 0];    % snap          (m/s5)

% transito obligado
mandatory = false;

end


methods


% function obj = Waypoint()
% end



function CheckWaypoint(~,wp)
    if ~isa(wp,'Waypoint')
       error('Error. \nValue must be a Waypoint object, not a %s.',class(wp))
    end
end



function Stop(obj)
    obj.vel  = [0 0 0];
    obj.acel = [0 0 0];
    obj.jerk = [0 0 0];
    obj.jolt = [0 0 0];
    obj.snap = [0 0 0];
end



function Postpone(obj,timeStep)
    % Set the position of the waypoint
    obj.t = obj.t + timeStep;
end



function time = TimeTo(a,b)
    % Get the time elapsed from this waypoint to another given
    a.CheckWaypoint(b);
    time = b.t - a.t;
end



function dist = DistanceTo(a,b)
    % Get the distance between two waypoints
    a.CheckWaypoint(b);
    dist = norm(a.pos - b.pos);
end



function dir = DirectionTo(a,b)
    % Get a direction vector from one waypoint to another            
    a.CheckWaypoint(b);
    dist = a.DistanceTo(b);
    if dist == 0
        dir = [0,0,0];
    else
        dir = (b.pos - a.pos) / dist;
    end
end



function angle = CourseTo(a,b)
    % Get the course from one waypoint to another
    % -X -> -  90
    % +Y ->     0
    % +X -> +  90
    % -Y -> +-180
    a.CheckWaypoint(b);
    dist = a.DistanceTo(b);
    if dist == 0
        angle = 0;
    else
        cx = b.pos(1) - a.pos(1);
        cy = b.pos(2) - a.pos(2);
        angle = atan2d(cx,cy);
    end
end



function angle = AngleWith(a,b)
    % Get the angle between the direction of two waypoints
    a.CheckWaypoint(b);

    angle = acos( dot(a.vel,b.vel) / (norm(a.vel) * norm(b.vel)) );
    % angled = rad2deg(angle);
end



function SetV0000(wp1,wp2)
    % Set uniform straight velocity from A to B
    wp1.CheckWaypoint(wp2);

    wp1.Stop();

    t12 = wp1.TimeTo(wp2);
    if t12 ~= 0
        wp1.vel = (wp2.pos - wp1.pos) / t12;
    end
    
end



function SetJLS(wp1,wp2)
    % Dados dos waypoints con 
    % tiempo, posición, velocidad y aceleración determinados
    % obtiene las 3 derivadas siguientes que ejecutan dicho movimiento

    wp1.CheckWaypoint(wp2);

    r1 = wp1.pos;
    v1 = wp1.vel;
    a1 = wp1.acel;
    r2 = wp2.pos; 
    v2 = wp2.vel;
    a2 = wp2.acel;

    if norm(r2-r1)==0
        wp1.Stop();
        return
    end
    
    t12 = wp1.TimeTo(wp2);

    A = [ t12^3/6   t12^4/24   t12^5/120 
          t12^2/2   t12^3/6    t12^4/24 
          t12       t12^2/2    t12^3/6  ];
    B = [ r2-r1-v1*t12 
          v2-v1  
          a2-a1            ];
    if rank(A) < 3
        error('Error. Interpolation not possible')
    end  

    X = A\B;
    wp1.jerk = X(1,:);
    wp1.jolt = X(2,:);
    wp1.snap = X(3,:);

end



function wp2 = Interpolation(wp1,t2)
    % Dados dos waypoints con tiempo, posición, velocidad y aceleración nula
    % interpola un tercer waypoint a un tiempo dado

    wp2 = Waypoint;
    wp2.t = t2;

    r1 = wp1.pos;
    v1 = wp1.vel;
    a1 = wp1.acel;
    j1 = wp1.jerk;
    l1 = wp1.jolt;
    s1 = wp1.snap;
    t12= wp1.TimeTo(wp2);
   
    r2 = r1 + v1*t12 + 1/2*a1*t12^2 + 1/6*j1*t12^3 + 1/24*l1*t12^4 + 1/120*s1*t12^5 ;
    v2 = v1 + a1*t12 + 1/2*j1*t12^2 + 1/6*l1*t12^3 + 1/24*s1*t12^4 ;
    a2 = a1 + j1*t12 + 1/2*l1*t12^2 + 1/6*s1*t12^3 ;
    j2 = j1 + l1*t12 + 1/2*s1*t12^2 ;
    l2 = l1 + s1*t12 ;
    s2 = s1 ;

    wp2.pos  = r2;
    wp2.vel  = v2;
    wp2.acel = a2;
    wp2.jerk = j2;
    wp2.jolt = l2;
    wp2.snap = s2;

end




end % methods
end % classdef 