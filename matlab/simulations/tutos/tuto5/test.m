clc
clear
run('../../../tools/NAVSIM_PATHS');

% -------------
% Flight Plan

% Create waypoints

wp1 = Waypoint();
wp1.label = 'wp1';
wp1.t = 0;
wp1.SetPosition([0 0 0]);
wp1.SetVelocity([10 0 0]);

wp2  = Waypoint();
wp2.label = 'wp2';
wp2.t = 10;
wp2.SetPosition([100 0 0]);
wp2.SetVelocity([0 10 0]);

wp3  = Waypoint();
wp3.label = 'wp3';
wp3.t = 20;
wp3.SetPosition([100 100 0]);
wp3.SetVelocity([0 10 0]);

wp4  = Waypoint();
wp4.label = 'wp4';
wp4.t = 30;
wp4.SetPosition([100 200 0]);
wp4.SetVelocity([0 0 0]);


% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.mode = InterpolationModes.TP;

fp1.SetWaypoint(wp1);
fp1.SetWaypoint(wp2);
fp1.SetWaypoint(wp3);
fp1.SetWaypoint(wp4);

fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


fp2 = fp1.Convert2TPV();
fp2.mode = InterpolationModes.TPV0;

fp2.ApplyDubinsAt('wp2',0.2);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);
 
