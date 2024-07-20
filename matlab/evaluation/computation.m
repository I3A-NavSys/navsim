clc
clear
run('../tools/NAVSIM_PATHS');

% -------------
% Vertiports location
%               x        y        z       rz
portsLoc = [ -190.00  -119.00  +048.00    pi/2  
             -152.00  -106.00  +049.00    pi/2  
             -200.00  -176.00  +010.00    pi/2
             +186.00  +195.00  +039.00    pi/2
             -200.00  -085.00  +029.00    pi/2];

vp1 = portsLoc(1,1:3);
vp2 = portsLoc(2,1:3);

vp3 = portsLoc(3,1:3);
vp4 = portsLoc(4,1:3);

vp5 = portsLoc(5,1:3);


% -------------
% Flight Plan

%% fp1 & fp2

% % Create waypoints
% wp1L = Waypoint;        % low waypoint
% wp1L.label = 'wp1L';
% 
% wp1P = Waypoint;        % pause waypoint
% wp1P.label = 'wp1P';
% 
% wp1 = Waypoint;
% wp1.label = 'wp1';
% 
% wp2  = Waypoint;
% wp2.label = 'wp2';
% 
% wp3  = Waypoint;
% wp3.label = 'wp3';
% 
% wp4  = Waypoint;
% wp4.label = 'wp4';
% 
% wp4L = Waypoint;
% wp4L.label = 'wp4L';
% 
% wp4P = Waypoint;
% wp4P.label = 'wp4P';
% 
% 
% % take off / landing positions
% wp1L.pos = vp1 + [0 0 0.10];
% wp1P.pos = wp1L.pos;
% wp4L.pos = vp2 + [0 0 0.10];
% wp4P.pos = wp4L.pos;
% 
% % take off / landing hovering positions
% wp1.pos = wp1L.pos;
% wp1.pos(3) = 70;
% wp4.pos = wp4L.pos;
% wp4.pos(3) = wp1.pos(3);
% 
% % route             
% wp2.pos = wp1.pos  + [   0  200 0];
% wp3.pos = wp2.pos  + [ 500    0 0];
% 
% % Compose the flight plan
% fp1  = FlightPlan(Waypoint.empty);
% fp1.radius = 2;
% fp1.AppendWaypoint(wp1L);
% wp1P.t = fp1.FinishTime + 5;
% fp1.SetWaypoint(wp1P);
% fp1.AppendWaypoint(wp1);
% fp1.AppendWaypoint(wp2);
% fp1.AppendWaypoint(wp3);
% fp1.AppendWaypoint(wp4);
% fp1.AppendWaypoint(wp4L);
% wp4P.t = fp1.FinishTime + 5;
% fp1.SetWaypoint(wp4P);
% 
% % waypoint time intervals
% % establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
% fp1.SetTimeFromVel('wp1' ,2);
% fp1.SetTimeFromVel('wp2' ,8);
% fp1.SetTimeFromVel('wp3' ,8);
% fp1.SetTimeFromVel('wp4' ,8);
% fp1.SetTimeFromVel('wp4L',2);
% 
% % Asignamos el vector velocidad de cada waypoint para conseguir un
% % movimiento rectilineo uniforme
% fp1.SetV0000;
% fp1.PositionFigure("FP1: POSITION",0.01);
% fp1.VelocityFigure("FP1: VELOCITY",0.01);
% 
% % Vamos a hacer una copia de la ruta anterior con intención de suavizarla
% fp2 = fp1.Copy;
% % Desplazamos ligeramente el par de puntos de inicio 
% % para que los dos drones no colisionen en el despegue
% fp2.waypoints(1).pos = fp2.waypoints(1).pos + [0.5 0 0];
% fp2.waypoints(2).pos = fp2.waypoints(2).pos + [0.5 0 0];
% fp2.PositionFigure("FP2: POSITION",0.1);
% fp2.VelocityFigure("FP2: VELOCITY",0.1);
% 
% %asumimos velocidad angular y aceleración lineal finitas
% ang_vel = 0.1;
% lin_acel =0.4;
% 
% % suavizamos 
% fp2.SmoothVertexMaintainingDuration('wp1P',ang_vel,lin_acel);
% fp2.PositionFigure("FP2: POSITION",0.1);
% fp2.VelocityFigure("FP2: VELOCITY",0.1);
% 
% fp2.SmoothVertexMaintainingDuration('wp1',ang_vel,lin_acel);
% fp2.PositionFigure("FP2: POSITION",0.1);
% fp2.VelocityFigure("FP2: VELOCITY",0.1);
% 
% fp2.SmoothVertexMaintainingSpeed('wp2',ang_vel);
% fp2.PositionFigure("FP2: POSITION",0.1);
% fp2.VelocityFigure("FP2: VELOCITY",0.1);
% 
% fp2.SmoothVertexMaintainingSpeed('wp3',ang_vel);
% fp2.PositionFigure("FP2: POSITION",0.1);
% fp2.VelocityFigure("FP2: VELOCITY",0.1);
% 
% fp2.SmoothVertexMaintainingDuration('wp4',ang_vel,lin_acel);
% fp2.PositionFigure("FP2: POSITION",0.01);
% fp2.VelocityFigure("FP2: VELOCITY",0.01);
% 
% fp2.SmoothVertexMaintainingDuration('wp4L',ang_vel,lin_acel);
% fp2.PositionFigure("FP2: POSITION",0.1);
% fp2.VelocityFigure("FP2: VELOCITY",0.1);


%% fp3 & fp4

% Create waypoints
% wp1L = Waypoint;        % low waypoint
% wp1L.label = 'wp1L';
% 
% wp1P = Waypoint;        % pause waypoint
% wp1P.label = 'wp1P';
% 
% wp1 = Waypoint;
% wp1.label = 'wp1';
% 
% wp2  = Waypoint;
% wp2.label = 'wp2';
% 
% wp3  = Waypoint;
% wp3.label = 'wp3';
% 
% wp3L = Waypoint;
% wp3L.label = 'wp3L';
% 
% wp3P = Waypoint;
% wp3P.label = 'wp3P';
% 
% 
% % take off / landing positions
% wp1L.pos = vp3 + [0 0 0.10];
% wp1P.pos = wp1L.pos;
% wp3L.pos = vp4 + [0 0 0.10];
% wp3P.pos = wp3L.pos;
% 
% % take off / landing hovering positions
% wp1.pos = wp1L.pos;
% wp1.pos(3) = 70;
% wp3.pos = wp3L.pos;
% wp3.pos(3) = wp1.pos(3);
% 
% % route             
% wp2.pos = wp1.pos  + [   0  371 0];
% wp3.pos = wp2.pos  + [ 386    0 0];
% 
% % Compose the flight plan
% fp3  = FlightPlan(Waypoint.empty);
% fp3.radius = 2;
% fp3.AppendWaypoint(wp1L);
% wp1P.t = fp3.FinishTime + 5;
% fp3.SetWaypoint(wp1P);
% fp3.AppendWaypoint(wp1);
% fp3.AppendWaypoint(wp2);
% fp3.AppendWaypoint(wp3);
% fp3.AppendWaypoint(wp3L);
% wp3P.t = fp3.FinishTime + 5;
% fp3.SetWaypoint(wp3P);
% 
% % waypoint time intervals
% % establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
% fp3.SetTimeFromVel('wp1' ,2);
% fp3.SetTimeFromVel('wp2' ,6);
% fp3.SetTimeFromVel('wp3' ,6);
% fp3.SetTimeFromVel('wp3L',2);
% 
% % Asignamos el vector velocidad de cada waypoint para conseguir un
% % movimiento rectilineo uniforme
% fp3.SetV0000;
% fp3.PositionFigure("FP3: POSITION",0.01);
% fp3.VelocityFigure("FP3: VELOCITY",0.01);
% 
% % Vamos a hacer una copia de la ruta anterior con intención de suavizarla
% fp4 = fp3.Copy;
% % Desplazamos ligeramente el par de puntos de inicio 
% % para que los dos drones no colisionen en el despegue
% fp4.waypoints(1).pos = fp4.waypoints(1).pos + [0.5 0 0];
% fp4.waypoints(2).pos = fp4.waypoints(2).pos + [0.5 0 0];
% fp4.PositionFigure("FP4: POSITION",0.1);
% fp4.VelocityFigure("FP4: VELOCITY",0.1);
% 
% %asumimos velocidad angular y aceleración lineal finitas
% ang_vel = 0.1;
% lin_acel =0.4;
% 
% % suavizamos 
% fp4.SmoothVertexMaintainingDuration('wp1P',ang_vel,lin_acel);
% fp4.PositionFigure("FP4: POSITION",0.1);
% fp4.VelocityFigure("FP4: VELOCITY",0.1);
% 
% fp4.SmoothVertexMaintainingDuration('wp1',ang_vel,lin_acel);
% fp4.PositionFigure("FP4: POSITION",0.1);
% fp4.VelocityFigure("FP4: VELOCITY",0.1);
% 
% fp4.SmoothVertexMaintainingSpeed('wp2',ang_vel);
% fp4.PositionFigure("FP4: POSITION",0.1);
% fp4.VelocityFigure("FP4: VELOCITY",0.1);
% 
% fp4.SmoothVertexMaintainingDuration('wp3',ang_vel,lin_acel);
% fp4.PositionFigure("FP4: POSITION",0.01);
% fp4.VelocityFigure("FP4: VELOCITY",0.01);
% 
% fp4.SmoothVertexMaintainingDuration('wp3L',ang_vel,lin_acel);
% fp4.PositionFigure("FP4: POSITION",0.1);
% fp4.VelocityFigure("FP4: VELOCITY",0.1);

%% fp5 & fp6

% Create waypoints
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
wp3.label = 'wp4';

wp5  = Waypoint;
wp3.label = 'wp5';

wp5L = Waypoint;
wp5L.label = 'wp5L';

wp5P = Waypoint;
wp5P.label = 'wp5P';


% take off / landing positions
wp1L.pos = vp5 + [0 0 0.10];
wp1P.pos = wp1L.pos;
wp5L.pos = vp5 + [0 0 0.10];
wp5P.pos = wp5L.pos;

% take off / landing hovering positions
wp1.pos = wp1L.pos;
wp1.pos(3) = 50;
wp5.pos = wp5L.pos;
wp5.pos(3) = wp1.pos(3);

% route             
wp2.pos = wp1.pos  + [  25    0    0];
wp3.pos = wp2.pos  + [  25    0    0];
wp4.pos = wp3.pos  + [  25    0    0];
wp5.pos = wp1.pos;

% Compose the flight plan
fp5  = FlightPlan(Waypoint.empty);
fp5.radius = 2;
fp5.AppendWaypoint(wp1L);
wp1P.t = fp5.FinishTime + 5;
fp5.SetWaypoint(wp1P);
fp5.AppendWaypoint(wp1);
fp5.AppendWaypoint(wp2);
fp5.AppendWaypoint(wp3);
fp5.AppendWaypoint(wp4);
fp5.AppendWaypoint(wp5);
fp5.AppendWaypoint(wp5L);
wp5P.t = fp5.FinishTime + 5;
fp5.SetWaypoint(wp5P);

% waypoint time intervals
% establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
fp5.SetTimeFromVel('wp1' ,2);
fp5.SetTimeFromVel('wp2' ,5);
fp5.SetTimeFromVel('wp3' ,10);
fp5.SetTimeFromVel('wp4' ,15);
fp5.SetTimeFromVel('wp5' ,5);
fp5.SetTimeFromVel('wp5L',2);

% Asignamos el vector velocidad de cada waypoint para conseguir un
% movimiento rectilineo uniforme
fp5.SetV0000;
fp5.PositionFigure("FP5: POSITION",0.01);
fp5.VelocityFigure("FP5: VELOCITY",0.01);

% Vamos a hacer una copia de la ruta anterior con intención de suavizarla
fp6 = fp5.Copy;
% Desplazamos ligeramente el par de puntos de inicio 
% para que los dos drones no colisionen en el despegue
fp6.waypoints(1).pos = fp6.waypoints(1).pos + [0.5 0 0];
fp6.waypoints(2).pos = fp6.waypoints(2).pos + [0.5 0 0];

fp6.PositionFigure("FP6: POSITION",0.1);
fp6.VelocityFigure("FP6: VELOCITY",0.1);

%asumimos velocidad angular y aceleración lineal finitas
ang_vel = 0.1;
lin_acel =0.4;

% suavizamos 
fp6.SmoothVertexMaintainingDuration('wp1P',ang_vel,lin_acel);
fp6.PositionFigure("FP6: POSITION",0.1);
fp6.VelocityFigure("FP6: VELOCITY",0.1);

fp6.SmoothVertexMaintainingDuration('wp1',ang_vel,lin_acel);
fp6.PositionFigure("FP6: POSITION",0.1);
fp6.VelocityFigure("FP6: VELOCITY",0.1);

fp6.SmoothVertexMaintainingSpeed('wp2',ang_vel);
fp6.PositionFigure("FP6: POSITION",0.1);
fp6.VelocityFigure("FP6: VELOCITY",0.1);

fp6.SmoothVertexMaintainingSpeed('wp3',ang_vel);
fp6.PositionFigure("FP6: POSITION",0.1);
fp6.VelocityFigure("FP6: VELOCITY",0.1);

fp6.SmoothVertexMaintainingSpeed('wp4',ang_vel);
fp6.PositionFigure("FP6: POSITION",0.1);
fp6.VelocityFigure("FP6: VELOCITY",0.1);

fp6.SmoothVertexMaintainingDuration('wp5',ang_vel,lin_acel);
fp6.PositionFigure("FP6: POSITION",0.01);
fp6.VelocityFigure("FP6: VELOCITY",0.01);

fp6.SmoothVertexMaintainingDuration('wp5L',ang_vel,lin_acel);
fp6.PositionFigure("FP6: POSITION",0.1);
fp6.VelocityFigure("FP6: VELOCITY",0.1);