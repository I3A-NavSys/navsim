clc
clear
run('../../../tools/NAVSIM_PATHS');


% -------------
% Flight Plan

wp1 = Waypoint;
wp1.label = 'wp1';
wp1.pos  = [ 00 00 00];
wp1.vel  = [ 10 00 00];
wp1.t    = 0;

wp2  = Waypoint;
wp2.label = 'wp2';
wp2.pos  = [ 100 100 0];
wp2.vel  = [ 00  10 00];


% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.SetWaypoint(wp1);
fp1.AppendWaypoint(wp2);


%% caso 1
wp2.t = 20

fp1.SetJLS();
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


%% caso 2
r = 100;
a = wp1.AngleWith(wp2);
d = r*a;
wp2.t = d / norm(wp1.vel)

fp1.SetJLS();
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


%% caso 3
wp2.t    = 16.4
fp1.SetJLS();
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


