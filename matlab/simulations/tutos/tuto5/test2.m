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
wpA0.pos = [0 0 0];


wpA1 = Waypoint();
wpA1.label = 'wpA1';
wpA1.t = 10;
wpA1.pos = [0 0 1000];

wp1 = Waypoint();
wp1.label = 'wp1';
wp1.t = 20;
wp1.pos = [1000 0 1000];

wp2  = Waypoint();
wp2.label = 'wp2';
wp2.t = 30;
wp2.pos = [1000 1000 1000];

wp3  = Waypoint();
wp3.label = 'wp3';
wp3.t = 38;
wp3.pos = [1000 2000 1000];

wp4  = Waypoint();
wp4.label = 'wp4';
wp4.t = 50;
wp4.pos = [1000 3000 1000];

wp5  = Waypoint();
wp5.label = 'wp5';
wp5.t = 60;
wp5.pos = [1000 4000 1000];

wp6  = Waypoint();
wp6.label = 'wp6';
wp6.t = 70;
wp6.pos = [0 4000 1000];

wpB1  = Waypoint();
wpB1.label = 'wpB1';
wpB1.t = 80;
wpB1.pos = [-1000 4000 1000];

wpB0  = Waypoint();
wpB0.label = 'wpB0';
wpB0.t = 90;
wpB0.pos = [-1000 4000 0];

wpBF  = Waypoint();
wpBF.label = 'wpBF';
wpBF.t = 100;
wpBF.pos = [-1000 4000 0];


% Compose the flight plan
fp1  = FlightPlan([wpA0 wpA1 wp1 wp2 wp3 wp4 wp5 wp6 wpB1 wpB0 wpBF]);
fp1.SetLinearMovement();

fp1.PositionFigure("FP1: POSITION",1);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


% Another plan
fp2 = fp1.Copy();
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wpA1',0.4,3);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wp1' ,0.4,3);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wp2',0.4,10);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wp3',0.4,10);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wp4',0.4,20);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wp5' ,0.4,10);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wpB1',0.4,10);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);

fp2.SmoothWaypoint('wpB0',0.4,20);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);
