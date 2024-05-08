classdef Waypoint < handle

properties


% instante de tiempo
% establecido desde un instante inicial prefijado
t    {mustBeNumeric}    % segundos a futuro

% etiqueta opcional para referirse al waypoint
label

% posicion en espacio 3D
% establecida sobre un eje de referencia prefijado en el escenario
x    {mustBeNumeric}    % metros hacia el este
y    {mustBeNumeric}    % metros hacia el norte
z    {mustBeNumeric}    % metros hacia arriba

% velocidad en espacio 3D
% establecida sobre un eje de referencia prefijado en el escenario
vx   {mustBeNumeric}    % metros/s hacia el este
vy   {mustBeNumeric}    % metros/s hacia el norte
vz   {mustBeNumeric}    % metros/s hacia arriba

% Waypoint de obligado tránsito (en un plan de vuelo)
% valor lógico (verdadero/falso)
mandatory logical;

end


methods



function obj = Waypoint()

    %WAYPOINT Constructor for the class
    obj.t  = 0;
    obj.label = '';
    obj.x  = 0;
    obj.y  = 0;
    obj.z  = 0;
    obj.vx = 0;
    obj.vy = 0;
    obj.vz = 0;
    obj.mandatory = true;

end



function CheckWaypoint(~,wp)
    if ~isa(wp,'Waypoint')
       error('Error. \nValue must be a Waypoint object, not a %s.',class(wp))
    end
end



function p = Position(obj)
    % Get the position of the waypoint
    p = [ obj.x  obj.y  obj.z ];
end



function SetPosition(obj,p)
    % Set the position of the waypoint
    obj.x = p(1);
    obj.y = p(2);
    obj.z = p(3);
end



function v = Velocity(obj)
    % Get the velocity of the waypoint
    v = [ obj.vx  obj.vy  obj.vz ];
end



function SetVelocity(obj,v)
    % Set the velocity of the waypoint
    obj.vx = v(1);
    obj.vy = v(2);
    obj.vz = v(3);
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
    dist = norm(a.Position - b.Position);
end



function dir = DirectionTo(a,b)
    % Get a direction vector from one waypoint to another            
    a.CheckWaypoint(b);
    dist = a.DistanceTo(b);
    if dist == 0
        dir = [0,0,0];
    else
        dir = (b.Position - a.Position) / dist;
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
        cx = b.x - a.x;
        cy = b.y - a.y;
        angle = atan2d(cx,cy);
    end
end



function vel = UniformVelocityTo(a,b)
    % Get the uniform velocity between two waypoints
    a.CheckWaypoint(b);

    dist = a.DistanceTo(b);
    time = a.TimeTo(b);

    if time == 0
        vel = 0;
    else
        vel = dist / time;
    end
end



function angle = AngleWith(a,b)
    % Get the angle between the direction of two waypoints
    a.CheckWaypoint(b);

    Va = a.Velocity();
    Vb = b.Velocity();
    angle = acos( dot(Va,Vb) / (norm(Va) * norm(Vb)) );
    % angled = rad2deg(angle);
end



function wp3 = InterpolationTP(wp1,wp2,t3)
    % Dados dos waypoints, 
    % genera un tercer waypoint interpolando posiciones a un tiempo dado.
    % Equivale a realizar un movimiento rectilíneo y uniforme.
    wp1.CheckWaypoint(wp2);
            
    wp3 = Waypoint;
    wp3.t = t3;

    r1 = wp1.Position;
    v1 = wp1.DirectionTo(wp2) * wp1.UniformVelocityTo(wp2);
    t = wp1.TimeTo(wp3);

    r3 = r1 + v1*t;
    v3 = v1;

    wp3.SetPosition(r3);
    wp3.SetVelocity(v3);

end



function wp3 = InterpolationTPV0(wp1,wp2,t3)
    % Dados dos waypoints con tiempo, posición, velocidad y aceleración nula
    % interpola un tercer waypoint a un tiempo dado

    wp1.CheckWaypoint(wp2);

    wp3 = Waypoint;
    wp3.t = t3;

    r1 = wp1.Position;
    v1 = wp1.Velocity;
    r2 = wp2.Position; 
    v2 = wp2.Velocity;


    if norm(r2-r1)==0
        wp3.SetPosition(wp1.Position);
        return
    end
    
    t12 = wp1.TimeTo(wp2);
    
    A = [ t12^3/6   t12^4/24   t12^5/120 
          t12^2/2   t12^3/6    t12^4/24 
          t12       t12^2/2    t12^3/6  ];

    B = [ r2-r1-v1*t12 
          v2-v1  
          0 0 0            ];

    if rank(A) < 3
        error('Error. Interpolación TPV sin solución')
    end  

    X  = A\B;
    x1 = X(1,:);  % jerk
    x2 = X(2,:);  % jolt
    x3 = X(3,:);  % jolt_dot

    t = wp1.TimeTo(wp3);
    r3 = r1 + v1*t              + 1/6*x1*t^3 + 1/24*x2*t^4 + 1/120*x3*t^5 ;
    v3 = v1        + 1/2*x1*t^2 + 1/6*x2*t^3 + 1/24*x3*t^4 ;
    a3 =      x1*t + 1/2*x2*t^2 + 1/6*x3*t^3 ;

    wp3.SetPosition(r3);
    wp3.SetVelocity(v3);

end



function [d1,d2,d3] = ResolveTPV0(wp1,wp2)
    % Dados dos waypoints con tiempo, posición, velocidad y aceleración nula 
    % obtiene las 3 derivadas siguientes que ejecutan dicho movimiento

    wp1.CheckWaypoint(wp2);

    r1 = wp1.Position;
    v1 = wp1.Velocity;
    r2 = wp2.Position; 
    v2 = wp2.Velocity;

    if norm(r2-r1)==0
        d1 = [0 0 0];  % jerk
        d2 = [0 0 0];  % jolt
        d3 = [0 0 0];  % jolt_dot
        return
    end
    
    t12 = wp1.TimeTo(wp2);

    A = [ t12^3/6   t12^4/24   t12^5/120 
          t12^2/2   t12^3/6    t12^4/24 
          t12       t12^2/2    t12^3/6  ];

    B = [ r2-r1-v1*t12 
          v2-v1  
          0 0 0            ];


    if rank(A) < 3
        error('Error. Interpolación TPV sin solución')
    end  


    X = A\B;
    d1 = X(1,:);  % jerk
    d2 = X(2,:);  % jolt
    d3 = X(3,:);  % jolt_dot

end


end % methods
end % classdef 