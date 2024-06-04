clc
clear
run('../../../tools/NAVSIM_PATHS');


% -------------
% Flight Plan

wp1 = Waypoint;
wp1.label = 'wp1';

wp2  = Waypoint;
wp2.label = 'wp2';

wp3  = Waypoint;
wp3.label = 'wp3';




% take off / landing positions
% Ubicamos un par de puntos 10 cms por encima del vertipuerto de despegue
wp1.pos = [   0    0 0];
wp2.pos = [ 100    0 0];
wp3.pos = [ 100  100 0];

% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.radius = 2;
fp1.AppendWaypoint(wp1);
fp1.AppendWaypoint(wp2);
fp1.AppendWaypoint(wp3);

% waypoint time intervals
% establecemos el tiempo de cada waypoint en función de la velocidad de desplazamiento deseada
fp1.SetTimeFromVel('wp2' ,10);
fp1.SetTimeFromVel('wp3' ,10);

% Asignamos el vector velocidad de cada waypoint para conseguir un
% movimiento rectilineo uniforme
fp1.SetV0000;
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);

% Vamos a hacer una copia de la ruta anterior con intención de suavizarla
fp2 = fp1.Copy;
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);



%asumimos velocidad angular y aceleración lineal finitas
ang_vel = 0.2;
lin_acel =0.4;

%suavizamos el paso por el waypoint 2
% fp2.SmoothVertexMaintainingDuration('wp2',ang_vel,lin_acel);
fp2.SmoothVertex('wp2',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);

fp2.VelocityFigure("FP2: VELOCITY",0.1);


figure(5)
tr = fp2.Trace(0.1);
plot(tr(:,2),tr(:,3),'-',LineWidth = 2,Color = 'blue');
viscircles([50 50],50)
axis equal
grid on
