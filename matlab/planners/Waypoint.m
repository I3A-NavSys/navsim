classdef Waypoint < handle
%WAYPOINT This class represent a flight plan's waypoint
%   Detailed explanation goes here

properties

% instante de tiempo
% establecido desde un instante inicial prefijado
t    {mustBeNumeric}    % segundos a futuro.

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
    obj.x  = 0;
    obj.y  = 0;
    obj.z  = 0;
    obj.vx = 0;
    obj.vy = 0;
    obj.vz = 0;
    obj.mandatory = true;

end



function CheckWaypoint(obj,wp)
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



function wp3 = InterpolationTP(wp1,wp2,t)
    % Dados dos waypoints, 
    % genera un tercer waypoint interpolando posiciones a un tiempo dado.
    % Equivale a realizar un movimiento rectilíneo y uniforme.
    wp1.CheckWaypoint(wp2);
            
    wp3 = Waypoint;
    wp3.t = t;
    s13 = wp1.DirectionTo(wp2) * wp1.UniformVelocityTo(wp2) * wp1.TimeTo(wp3);
    wp3.SetPosition(wp1.Position + s13);

end



function wp3 = InterpolationTPV(wp1,wp2,t)
    % PENDIENTE DE CHEQUEAR
    % Dados dos waypoints, 
    % genera un tercer waypoint interpolando posiciones y velocidades a un tiempo dado.
    wp1.CheckWaypoint(wp2);

    wp3 = Waypoint;
    wp3.t = t;

    % s12 = wp1.distanceTo(wp2);
    s12 = wp2.Position - wp1.Position;
    if norm(s12)==0
        wp3.SetPosition(wp1.Position);
        return
    end
    
    t12 = wp1.TimeTo(wp2);
    t13 = wp1.TimeTo(wp3);
    v1  = wp1.Velocity;
    v2  = wp2.Velocity;
    
    A = [ t12^2/2   t12^3/6 ;
          t12       t12^2/2 ];

    B = [ s12-v1*t12 ;
          v2-v1      ];

    if rank(A) == 2
        X = A\B;
        a = X(1,:);
        j = X(2,:);
    else
        error('Error. Interpolación TPV sin solución')
    end  

    s13 = v1*t13 + 1/2 *a*t13^2 + 1/6 *j*t13^3;
    wp3.SetPosition( wp1.Position + s13 );

end



end % methods
end % classdef 