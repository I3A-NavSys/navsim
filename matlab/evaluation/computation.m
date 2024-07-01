clc
clear
run('../tools/NAVSIM_PATHS');

% -------------
% Vertiports location
%               x        y        z       rz
portsLoc = [ -190.00  -119.00  +048.00    pi/2  
             -152.00  -106.00  +049.00    pi/2   ];
vp1 = portsLoc(1,1:3);
vp2 = portsLoc(2,1:3);


% -------------
% Flight Plan

% Create waypoints
% Definiremos 6 waypoints a una altura determinada. 
% Además, el primero y el último tendran otros dos puntos debajo de ellos en las zonas de despegue y aterrizaje.
% Cada par esta ubicado en el mismo lugar, pero en distinto tiempo (para que el dron haga una pausa).

wp1L = Waypoint;        % low waypoint
wp1L.label = 'wp1L';

wp1P = Waypoint;        % pause waypoint
wp1P.label = 'wp1P';

wp1 = Waypoint;
wp1.label = 'wp1';

wp2  = Waypoint;
wp2.label = 'wp2';

wp3  = Waypoint;
wp3.label = 'wp3';

wp4  = Waypoint;
wp4.label = 'wp4';

% wp5  = Waypoint;
% wp5.label = 'wp5';
% 
% wp6 = Waypoint;
% wp6.label = 'wp6';

wp4L = Waypoint;
wp4L.label = 'wp4L';

wp4P = Waypoint;
wp4P.label = 'wp4P';


% take off / landing positions
% Ubicamos un par de puntos 10 cms por encima del vertipuerto de despegue
wp1L.pos = vp1 + [0 0 0.10];
wp1P.pos = wp1L.pos;
% Ubicamos un par de puntos 10 cms por encima del vertipuerto de aterrizaje
wp4L.pos = vp2 + [0 0 0.10];
wp4P.pos = wp4L.pos;

% take off / landing hovering positions
wp1.pos = wp1L.pos;
wp1.pos(3) = 70;
wp4.pos = wp4L.pos;
wp4.pos(3) = wp1.pos(3);

% route             
wp2.pos = wp1.pos  + [   0  200 0];
wp3.pos = wp2.pos  + [ 500    0 0];
%wp4.pos = wp1.pos;

% % desplazamos los puntos de inicio y fin de ruta 20 metros
% % para que el despegue y el aterrizaje no sean completamente verticales
% wp1.pos = wp1.pos + 20 * wp1.DirectionTo(wp2);
% wp6.pos = wp6.pos + 20 * wp6.DirectionTo(wp5);

% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.radius = 2;
fp1.AppendWaypoint(wp1L);
wp1P.t = fp1.FinishTime + 5;
fp1.SetWaypoint(wp1P);
fp1.AppendWaypoint(wp1);
fp1.AppendWaypoint(wp2);
fp1.AppendWaypoint(wp3);
fp1.AppendWaypoint(wp4);
fp1.AppendWaypoint(wp4L);
wp4P.t = fp1.FinishTime + 5;
fp1.SetWaypoint(wp4P);

% waypoint time intervals
% establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
fp1.SetTimeFromVel('wp1' ,2);
fp1.SetTimeFromVel('wp2' ,8);
fp1.SetTimeFromVel('wp3' ,8);
fp1.SetTimeFromVel('wp4' ,8);
fp1.SetTimeFromVel('wp4L',2);

% Asignamos el vector velocidad de cada waypoint para conseguir un
% movimiento rectilineo uniforme
fp1.SetV0000;
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


% Vamos a hacer una copia de la ruta anterior con intención de suavizarla
fp2 = fp1.Copy;
% Desplazamos ligeramente el par de puntos de inicio 
% para que los dos drones no colisionen en el despegue
fp2.waypoints(1).pos = fp2.waypoints(1).pos + [0.5 0 0];
fp2.waypoints(2).pos = fp2.waypoints(2).pos + [0.5 0 0];
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);



%asumimos velocidad angular y aceleración lineal finitas
ang_vel = 0.1;
lin_acel =0.4;

% suavizamos 

fp2.SmoothVertexMaintainingDuration('wp1P',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);


fp2.SmoothVertexMaintainingDuration('wp1',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);


fp2.SmoothVertexMaintainingSpeed('wp2',ang_vel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

fp2.SmoothVertexMaintainingSpeed('wp3',ang_vel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

fp2.SmoothVertexMaintainingDuration('wp4',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothVertexMaintainingDuration('wp4L',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

