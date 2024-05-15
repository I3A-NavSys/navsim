clc
clear
run('../../../tools/NAVSIM_PATHS');

% -------------
% UAV performance
info = UAVinfo('',UAVmodels.MiniDroneFP1);
info.maxForwardVel   = 12;    % m/s
info.maxForwardAcel  =  1;    % m/s2
info.maxVerticalVel  =  3;    % m/s  
info.maxVerticalAcel =  0;    % m/s2
info.maxAngularVel   =  5;    % rad/s
info.maxAngularAcel  =  1;    % rad/s2

% -------------
% Vertiports location
%               x        y        z       rz
portsLoc = [ -190.00  -119.00  +048.00    pi/2
             -152.00  -106.00  +049.00    pi/2   ];
vp1 = portsLoc(1,1:3);
vp2 = portsLoc(2,1:3);


% -------------
% Flight Plan

% Create waypoints

wp1L = Waypoint();
wp1L.label = 'wp1L';

wp1 = Waypoint();
wp1.label = 'wp1';

wp2  = Waypoint();
wp2.label = 'wp2';

wp3  = Waypoint();
wp3.label = 'wp3';

wp4  = Waypoint();
wp4.label = 'wp4';

wp5  = Waypoint();
wp5.label = 'wp5';

wp6 = Waypoint();
wp6.label = 'wp6';

wp6L = Waypoint();
wp6L.label = 'wp6L';

% take off / landing positions
wp1L.pos = vp1 + [0 0 0.10];
wp6L.pos = vp2 + [0 0 0.10];

% take off / landing hovering positions
wp1.pos = wp1L.pos;
wp1.pos(3) = 70;
wp6.pos = wp6L.pos;
wp6.pos(3) = wp1.pos(3);

% route             
wp2.pos = wp1.pos  + [ 100  100 0];
wp3.pos = wp2.pos  + [ 100    0 0];
wp4.pos = wp3.pos  + [   0 -100 0];
wp5.pos = wp4.pos  + [-100 -100 0];

wp1.pos = wp1.pos + 20 * wp1.DirectionTo(wp2);
wp6.pos = wp6.pos - 20 * wp5.DirectionTo(wp6);


% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.InsertWaypoint(wp1L);
fp1.InsertWaypoint(wp1);
fp1.InsertWaypoint(wp2);
fp1.InsertWaypoint(wp3);
fp1.InsertWaypoint(wp4);
fp1.InsertWaypoint(wp5);
fp1.InsertWaypoint(wp6);
fp1.InsertWaypoint(wp6L);

% waypoint time intervals
fp1.SetTimeFromVel('wp1' ,2);
fp1.SetTimeFromVel('wp2' ,8);
fp1.SetTimeFromVel('wp3' ,10);
fp1.SetTimeFromVel('wp4' ,12);
fp1.SetTimeFromVel('wp5' ,10);
fp1.SetTimeFromVel('wp6' ,8);
fp1.SetTimeFromVel('wp6L',2);

fp1.SetLinearMovement;
fp1.PositionFigure("FP1: POSITION",0.01);
fp1.VelocityFigure("FP1: VELOCITY",0.01);


fp2 = fp1.Copy();
fp2.waypoints(1).pos = fp2.waypoints(1).pos - [0.5 0 0];
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

ang_vel = 0.5;
lin_acel =0.4;
fp2.SmoothWaypoint('wp1',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);
 
fp2.SmoothWaypoint('wp2',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

fp2.SmoothWaypoint('wp3',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

fp2.SmoothWaypoint('wp4',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

fp2.SmoothWaypoint('wp5',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

fp2.SmoothWaypoint('wp6',ang_vel,lin_acel);
fp2.PositionFigure("FP2: POSITION",0.01);
fp2.VelocityFigure("FP2: VELOCITY",0.01);
 

% -------------
% Simulation
builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
pause(0.1)
operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
pause(0.1)
monitor  = SimpleMonitor('monitor');
pause(0.1)

% vertiports
for i = 1:size(portsLoc,1)
   
    id = sprintf('BASE%02d', i);
    builder.DeployModel('UAM/vertiport_H', id, ...
        portsLoc(i,1:3), ...
        [0 0 portsLoc(i,4)]);
    operator.SetVertiport(id,portsLoc(i,1:3),1);

end

% Deploy UAVs
operator.DeployUAV(info,'UAV01', ...
    [ -190.00  -119.00  +048.10 ],...
    [    0.00     0.00     pi/4  ]);
monitor.TrackUAV('UAV01');

operator.DeployUAV(info,'UAV02', ...
    [ -190.50  -119.00  +048.10 ],...
    [    0.00     0.00     pi/4 ]);
monitor.TrackUAV('UAV02');



time = operator.GetTime();

fp1.RescheduleAt(time + 10);
operator.SendFlightPlan('UAV01',fp1);

fp2.RescheduleAt(time + 10.1);
fp3 = fp2.Convert2TP(0.25);
operator.SendFlightPlan('UAV02',fp3);


operator.WaitTime(fp3.FinishTime);
monitor.PositionFigure('UAV01',fp1,0.01);
monitor.VelocityFigure('UAV01',fp1,0.01);
monitor.PositionFigure('UAV02',fp2,0.01);
monitor.VelocityFigure('UAV02',fp2,0.01);


operator.PauseSim;



