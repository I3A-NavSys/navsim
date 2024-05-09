clc
clear
run('../../../tools/NAVSIM_PATHS');

% -------------
% Flight Plan
% ejemplo de maniobra con varios waypoints

% Create waypoints

wpA0 = Waypoint();
wpA0.label = 'wpA0';
wpA0.t = 0;
wpA0.SetPosition([0 0 0]);


wpA1 = Waypoint();
wpA1.label = 'wpA1';
wpA1.t = 10;
wpA1.SetPosition([0 0 1000]);

wp1 = Waypoint();
wp1.label = 'wp1';
wp1.t = 20;
wp1.SetPosition([1000 0 1000]);

wp2  = Waypoint();
wp2.label = 'wp2';
wp2.t = 30;
wp2.SetPosition([1000 1000 1000]);

wp3  = Waypoint();
wp3.label = 'wp3';
wp3.t = 40;
wp3.SetPosition([1000 2000 1000]);

wp4  = Waypoint();
wp4.label = 'wp4';
wp4.t = 50;
wp4.SetPosition([1000 3000 1000]);

wp5  = Waypoint();
wp5.label = 'wp5';
wp5.t = 60;
wp5.SetPosition([1000 4000 1000]);

wp6  = Waypoint();
wp6.label = 'wp6';
wp6.t = 70;
wp6.SetPosition([0 4000 1000]);

wpB1  = Waypoint();
wpB1.label = 'wpB1';
wpB1.t = 80;
wpB1.SetPosition([-1000 4000 1000]);

wpB0  = Waypoint();
wpB0.label = 'wpB0';
wpB0.t = 90;
wpB0.SetPosition([-1000 4000 0]);

wpBF  = Waypoint();
wpBF.label = 'wpBF';
wpBF.t = 100;
wpBF.SetPosition([-1000 4000 0]);


% Compose the flight plan
fp1  = FlightPlan([wpA0 wpA1 wp1 wp2 wp3 wp4 wp5 wp6 wpB1 wpB0 wpBF]);
fp1.SetUniformVelocities();

fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


% Another plan
fp2 = fp1.Copy();
fp2.mode = InterpolationModes.TPV0;
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);


fp2.SmoothVertex('wpA1',0.4);
fp2.SmoothVertex('wp1',0.4);
fp2.SmoothVertex('wp5',0.4);
fp2.SmoothVertex('wpB1',0.4);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);


fp2.waypoints(1).SetVelocity([0 0 0]);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);


fp2.waypoints(14).t = 100;
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);