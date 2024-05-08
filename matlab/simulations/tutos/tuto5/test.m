clc
clear
run('../../../tools/NAVSIM_PATHS');

% -------------
% Flight Plan

% Create waypoints

wp1 = Waypoint();
wp1.label = 'wp1';
wp1.SetPosition([ 0  0  0 ]);
wp1.SetVelocity([ 2  0  0 ]);
wp1.t = 0;

wp2  = Waypoint();
wp2.label = 'wp2';
wp2.SetPosition([ 10 10  0 ]);
wp2.SetVelocity([  0  2  0 ]);
wp2.t = 8;


[d1,d2,d3] = wp1.ResolveTPV0(wp2)
wp = wp1.InterpolationTPV0(wp2,0)


% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.mode = InterpolationModes.TPV0;
fp1.SetWaypoint(wp1);
fp1.SetWaypoint(wp2);

% fp2.ApplyDubinsAt('wp2',0.2);
fp1.PositionFigure("FP2: POSITION",0.01);
fp1.VelocityFigure("FP2: VELOCITY",0.01);
 
