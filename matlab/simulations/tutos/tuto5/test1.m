clc
clear
run('../../../tools/NAVSIM_PATHS');

% -------------
% Flight Plan
% ejemplo de maniobra de giro en angulo recto
% mantenemos tiempo de la maniobra reduciendo la velocidad

% Create waypoints

wp1 = Waypoint();
wp1.label = 'wp1';
wp1.pos = [  0  0  0 ];
wp1.vel = [ 10  0  0 ];
wp1.t = 0;

wp2  = Waypoint();
wp2.label = 'wp2';
wp2.pos = [ 100 100  0 ];
wp2.vel = [   0  10  0 ];
wp2.t = 20;


wp1.SetFlyableMovement(wp2)

% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.SetWaypoint(wp1);
fp1.SetWaypoint(wp2);

% fp2.ApplyDubinsAt('wp2',0.2);
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);
 
