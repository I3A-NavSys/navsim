classdef Pose5D
properties
    % instante de tiempo
    % establecido desde un instante inicial prefijado
    t    {mustBeNumeric}    % segundos a futuro.

    % posicion en espacio 3D
    % establecida sobre un eje de referencia prefijado en el escenario
    x    {mustBeNumeric}    % metros hacia el este
    y    {mustBeNumeric}    % metros hacia el norte
    z    {mustBeNumeric}    % metros hacia arriba

    % velocidad de desplazamiento
    % expresada en modulo, sin direccion en espacio 3D
    v    {mustBeNumeric}    % metros/s
   
end %properties

methods

function obj = Pose5D
    obj.t  = NaN;
    obj.x  = NaN;  obj.y  = NaN; obj.z  = NaN;
    obj.v  = NaN;
end

function obj = setPose4D(obj,t,x,y,z)
    obj.t  = t;
    obj.x  = x;    obj.y  = y;   obj.z  = z;  
    obj.v  = NaN;
end

function obj = setPose5D(obj,t,x,y,z,v)
    obj.t  = t;
    obj.x  = x;    obj.y  = y;   obj.z  = z;  
    obj.v  = v;
end


function d = distance(obj,p2)
    d = sqrt((p2.x - obj.x)^2 + (p2.y - obj.y)^2 + (p2.z - obj.z)^2);
end


function t = delay(obj,p2)
    t = p2.t - obj.t;
end


function p3 = interpolation4D(obj,p2,t)
    p3 = Pose5D;

    s12 = obj.distance(p2);
    if s12==0
        p3 = obj;
        return
    end

    t12 = p2.t - obj.t;
    t13 =    t - obj.t;
    v1  = s12 / t12;
    
    s13 = v1*t13;

    p3.x = obj.x + (p2.x-obj.x) * s13 / s12;
    p3.y = obj.y + (p2.y-obj.y) * s13 / s12;
    p3.z = obj.z + (p2.z-obj.z) * s13 / s12;
    p3.t = obj.t + (p2.t-obj.t) * s13 / s12;
end

function p3 = interpolation5D(obj,p2,t)
    p3 = Pose5D;

    s12 = obj.distance(p2);
    if s12==0
        p3 = obj;
        return
    end
    
    t12 = p2.t - obj.t;
    t13 =    t - obj.t;
    v1  = obj.v;
    v2  = p2.v;
    
    A = [ t12^2/2   t12^3/6 ;
          t12       t12^2/2 ];
    B = [ s12-v1*t12 ;
          v2-v1    ];
    if rank(A) == 2
        X = A\B;
        a = X(1);
        j = X(2);
    else
        disp('NO existe solución')
        return
    end  

    s13 = v1*t13 + 1/2 *a*t13^2 + 1/6 *j*t13^3;

    p3.x = obj.x + (p2.x-obj.x) * s13 / s12;
    p3.y = obj.y + (p2.y-obj.y) * s13 / s12;
    p3.z = obj.z + (p2.z-obj.z) * s13 / s12;
    p3.t = obj.t + (p2.t-obj.t) * s13 / s12;
    
end

end %methods
end %class


