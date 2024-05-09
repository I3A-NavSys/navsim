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
wp1.SetPosition([  0  0  0 ]);
wp1.SetVelocity([ 10  0  0 ]);
wp1.t = 0;

wp2  = Waypoint();
wp2.label = 'wp2';
wp2.SetPosition([ 100 100  0 ]);
wp2.SetVelocity([   0  10  0 ]);
wp2.t = 20;


[d1,d2,d3] = wp1.ResolveTPV0(wp2)
wp = wp1.InterpolationTPV0(wp2,0)

% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.mode = InterpolationModes.TPV0;
fp1.SetWaypoint(wp1);
fp1.SetWaypoint(wp2);

% fp2.ApplyDubinsAt('wp2',0.2);
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);
 