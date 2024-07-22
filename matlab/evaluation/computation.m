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

%% fp1 & fp2: triángulo entre vp1 y vp2

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


%% fp3 & fp4: un ángulo recto entre vp3 y vp4

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

%% fp5 & fp6: escalonado entre vp3 y vp4

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
% wp3.label = 'wp4';
% 
% wp5  = Waypoint;
% wp3.label = 'wp5';
% 
% wp6  = Waypoint;
% wp6.label = 'wp6';
% 
% wp7  = Waypoint;
% wp7.label = 'wp7';
% 
% wp8  = Waypoint;
% wp8.label = 'wp8';
% 
% wp9  = Waypoint;
% wp9.label = 'wp9';
% 
% wp9L = Waypoint;
% wp9L.label = 'wp9L';
% 
% wp9P = Waypoint;
% wp9P.label = 'wp9P';
% 
% % take off / landing positions
% wp1L.pos = vp3 + [0 0 0.10];
% wp1P.pos = wp1L.pos;
% wp9L.pos = vp4 + [0 0 0.10];
% wp9P.pos = wp9L.pos;
% 
% % take off / landing hovering positions
% wp1.pos = wp1L.pos;
% wp1.pos(3) = 70;
% wp9.pos = wp9L.pos;
% wp9.pos(3) = wp1.pos(3);
% 
% % route             
% wp2.pos    = wp1.pos;
% wp2.pos(2) = -100;
% wp3.pos    = wp2.pos;
% wp3.pos(1) = -100;
% wp4.pos    = wp3.pos;
% wp4.pos(2) = 0;
% wp5.pos    = wp4.pos;
% wp5.pos(1) = 0;
% wp6.pos    = wp5.pos;
% wp6.pos(2) = 100;
% wp7.pos    = wp6.pos;
% wp7.pos(1) = 100;
% wp8.pos    = wp7.pos;
% wp8.pos(2) = 195;
% wp9.pos    = wp8.pos;
% wp9.pos(1) = 186;
% 
% % Compose the flight plan
% fp5  = FlightPlan(Waypoint.empty);
% fp5.radius = 5;
% fp5.AppendWaypoint(wp1L);
% wp1P.t = fp5.FinishTime + 5;
% fp5.SetWaypoint(wp1P);
% fp5.AppendWaypoint(wp1);
% fp5.AppendWaypoint(wp2);
% fp5.AppendWaypoint(wp3);
% fp5.AppendWaypoint(wp4);
% fp5.AppendWaypoint(wp5);
% fp5.AppendWaypoint(wp6);
% fp5.AppendWaypoint(wp7);
% fp5.AppendWaypoint(wp8);
% fp5.AppendWaypoint(wp9);
% fp5.AppendWaypoint(wp9L);
% wp9P.t = fp5.FinishTime + 5;
% fp5.SetWaypoint(wp9P);
% 
% % waypoint time intervals
% % establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
% fp5.SetTimeFromVel('wp1' ,1);
% fp5.SetTimeFromVel('wp2' ,2);
% fp5.SetTimeFromVel('wp3' ,2);
% fp5.SetTimeFromVel('wp4' ,2);
% fp5.SetTimeFromVel('wp5' ,2);
% fp5.SetTimeFromVel('wp6' ,2);
% fp5.SetTimeFromVel('wp7' ,2);
% fp5.SetTimeFromVel('wp8' ,2);
% fp5.SetTimeFromVel('wp9' ,2);
% fp5.SetTimeFromVel('wp9L',1);
% 
% % Asignamos el vector velocidad de cada waypoint para conseguir un
% % movimiento rectilineo uniforme
% fp5.SetV0000;
% fp5.PositionFigure("FP5: POSITION",0.01);
% fp5.VelocityFigure("FP5: VELOCITY",0.01);
% 
% % Vamos a hacer una copia de la ruta anterior con intención de suavizarla
% fp6 = fp5.Copy;
% % Desplazamos ligeramente el par de puntos de inicio 
% % para que los dos drones no colisionen en el despegue
% fp6.waypoints(1).pos = fp6.waypoints(1).pos + [0.5 0 0];
% fp6.waypoints(2).pos = fp6.waypoints(2).pos + [0.5 0 0];
% 
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% %asumimos velocidad angular y aceleración lineal finitas
% ang_vel = 0.1;
% lin_acel =0.4;
% 
% % suavizamos 
% fp6.SmoothVertexMaintainingDuration('wp1P',ang_vel,lin_acel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingDuration('wp1',ang_vel,lin_acel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingSpeed('wp2',ang_vel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingSpeed('wp3',ang_vel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingSpeed('wp4',ang_vel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingSpeed('wp5',ang_vel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingSpeed('wp6',ang_vel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingSpeed('wp7',ang_vel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingSpeed('wp8',ang_vel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);
% 
% fp6.SmoothVertexMaintainingDuration('wp9',ang_vel,lin_acel);
% fp6.PositionFigure("FP6: POSITION",0.01);
% fp6.VelocityFigure("FP6: VELOCITY",0.01);
% 
% fp6.SmoothVertexMaintainingDuration('wp9L',ang_vel,lin_acel);
% fp6.PositionFigure("FP6: POSITION",0.1);
% fp6.VelocityFigure("FP6: VELOCITY",0.1);

%% fp7 y fp8: barrido por columnas de vp3 a vp4

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
% wp3.label = 'wp4';
% 
% wp5  = Waypoint;
% wp3.label = 'wp5';
% 
% wp6  = Waypoint;
% wp6.label = 'wp6';
% 
% wp7  = Waypoint;
% wp7.label = 'wp7';
% 
% wp8  = Waypoint;
% wp8.label = 'wp8';
% 
% wp9  = Waypoint;
% wp9.label = 'wp9';
% 
% wp10  = Waypoint;
% wp10.label = 'wp10';
% 
% wp10L = Waypoint;
% wp10L.label = 'wp10L';
% 
% wp10P = Waypoint;
% wp10P.label = 'wp10P';
% 
% % take off / landing positions
% wp1L.pos = vp3 + [0 0 0.10];
% wp1P.pos = wp1L.pos;
% wp10L.pos = vp4 + [0 0 0.10];
% wp10P.pos = wp10L.pos;
% 
% % take off / landing hovering positions
% wp1.pos = wp1L.pos;
% wp1.pos(3) = 70;
% wp10.pos = wp10L.pos;
% wp10.pos(3) = wp1.pos(3);
% 
% % route             
% wp2.pos    = wp1.pos;
% wp2.pos(2) = 195;
% wp3.pos    = wp2.pos;
% wp3.pos(1) = -100;
% wp4.pos    = wp3.pos;
% wp4.pos(2) = -176;
% wp5.pos    = wp4.pos;
% wp5.pos(1) = 0;
% wp6.pos    = wp5.pos;
% wp6.pos(2) = 195;
% wp7.pos    = wp6.pos;
% wp7.pos(1) = 100;
% wp8.pos    = wp7.pos;
% wp8.pos(2) = -176;
% wp9.pos    = wp8.pos;
% wp9.pos(1) = 186;
% wp10.pos   = wp9.pos;
% wp10.pos(2)= 195;
% 
% 
% % Compose the flight plan
% fp7  = FlightPlan(Waypoint.empty);
% fp7.radius = 5;
% fp7.AppendWaypoint(wp1L);
% wp1P.t = fp7.FinishTime + 5;
% fp7.SetWaypoint(wp1P);
% fp7.AppendWaypoint(wp1);
% fp7.AppendWaypoint(wp2);
% fp7.AppendWaypoint(wp3);
% fp7.AppendWaypoint(wp4);
% fp7.AppendWaypoint(wp5);
% fp7.AppendWaypoint(wp6);
% fp7.AppendWaypoint(wp7);
% fp7.AppendWaypoint(wp8);
% fp7.AppendWaypoint(wp9);
% fp7.AppendWaypoint(wp10);
% fp7.AppendWaypoint(wp10L);
% wp10P.t = fp7.FinishTime + 5;
% fp7.SetWaypoint(wp10P);
% 
% % waypoint time intervals
% % establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
% fp7.SetTimeFromVel('wp1'  ,1);
% fp7.SetTimeFromVel('wp2'  ,2);
% fp7.SetTimeFromVel('wp3'  ,2);
% fp7.SetTimeFromVel('wp4'  ,2);
% fp7.SetTimeFromVel('wp5'  ,2);
% fp7.SetTimeFromVel('wp6'  ,2);
% fp7.SetTimeFromVel('wp7'  ,2);
% fp7.SetTimeFromVel('wp8'  ,2);
% fp7.SetTimeFromVel('wp9'  ,2);
% fp7.SetTimeFromVel('wp10' ,2);
% fp7.SetTimeFromVel('wp10L',1);
% 
% % Asignamos el vector velocidad de cada waypoint para conseguir un
% % movimiento rectilineo uniforme
% fp7.SetV0000;
% fp7.PositionFigure("FP7: POSITION",0.01);
% fp7.VelocityFigure("FP7: VELOCITY",0.01);
% 
% % Vamos a hacer una copia de la ruta anterior con intención de suavizarla
% fp8 = fp7.Copy;
% % Desplazamos ligeramente el par de puntos de inicio 
% % para que los dos drones no colisionen en el despegue
% fp8.waypoints(1).pos = fp8.waypoints(1).pos + [0.5 0 0];
% fp8.waypoints(2).pos = fp8.waypoints(2).pos + [0.5 0 0];
% 
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% %asumimos velocidad angular y aceleración lineal finitas
% ang_vel = 0.1;
% lin_acel =0.4;
% 
% % suavizamos 
% fp8.SmoothVertexMaintainingDuration('wp1P',ang_vel,lin_acel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingDuration('wp1',ang_vel,lin_acel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp2',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp3',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp4',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp5',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp6',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp7',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp8',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingSpeed('wp9',ang_vel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingDuration('wp10',ang_vel,lin_acel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);
% 
% fp8.SmoothVertexMaintainingDuration('wp10L',ang_vel,lin_acel);
% fp8.PositionFigure("FP8: POSITION",0.1);
% fp8.VelocityFigure("FP8: VELOCITY",0.1);

%% fp5 & fp6: despega/aterriza en vp5, con cambios de velocidad en el eje x

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
wp1.pos(3) = 70;
wp5.pos = wp5L.pos;
wp5.pos(3) = wp1.pos(3);

% route             
wp2.pos = wp1.pos  + [ 100    0    0];
wp3.pos = wp2.pos  + [ 100    0    0];
wp4.pos = wp3.pos  + [ 100    0    0];
wp5.pos = wp1.pos;

% Compose the flight plan
fp9  = FlightPlan(Waypoint.empty);
fp9.radius = 2;
fp9.AppendWaypoint(wp1L);
wp1P.t = fp9.FinishTime + 5;
fp9.SetWaypoint(wp1P);
fp9.AppendWaypoint(wp1);
fp9.AppendWaypoint(wp2);
fp9.AppendWaypoint(wp3);
fp9.AppendWaypoint(wp4);
fp9.AppendWaypoint(wp5);
fp9.AppendWaypoint(wp5L);
wp5P.t = fp9.FinishTime + 5;
fp9.SetWaypoint(wp5P);

% waypoint time intervals
% establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
fp9.SetTimeFromVel('wp1' ,2);
fp9.SetTimeFromVel('wp2' ,5);
fp9.SetTimeFromVel('wp3' ,5);
fp9.SetTimeFromVel('wp4' ,5);
fp9.SetTimeFromVel('wp5' ,5);
fp9.SetTimeFromVel('wp5L',2);

% Asignamos el vector velocidad de cada waypoint para conseguir un
% movimiento rectilineo uniforme
fp9.SetV0000;
fp9.PositionFigure("FP9: POSITION",0.01);
fp9.VelocityFigure("FP9: VELOCITY",0.01);

% Vamos a hacer una copia de la ruta anterior con intención de suavizarla
fp10 = fp9.Copy;
% Desplazamos ligeramente el par de puntos de inicio 
% para que los dos drones no colisionen en el despegue
fp10.waypoints(1).pos = fp10.waypoints(1).pos + [0.5 0 0];
fp10.waypoints(2).pos = fp10.waypoints(2).pos + [0.5 0 0];

fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);

%asumimos velocidad angular y aceleración lineal finitas
ang_vel = 0.1;
lin_acel =0.4;

% suavizamos 
fp10.SmoothVertexMaintainingDuration('wp1P',ang_vel,lin_acel);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);

fp10.SmoothVertexMaintainingDuration('wp1',ang_vel,lin_acel);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);

fp10.SmoothVertexMaintainingSpeed('wp2',ang_vel);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);

fp10.SmoothVertexMaintainingSpeed('wp3',ang_vel);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);

fp10.SmoothVertexMaintainingSpeed('wp4',ang_vel);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);

%fp10.SmoothVertexMaintainingDuration('wp5',ang_vel,lin_acel);
fp10.SmoothVertexMaintainingSpeed('wp5',ang_vel);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);

fp10.SmoothVertexMaintainingDuration('wp5L',ang_vel,lin_acel);
fp10.PositionFigure("FP10: POSITION",0.1);
fp10.VelocityFigure("FP10: VELOCITY",0.1);