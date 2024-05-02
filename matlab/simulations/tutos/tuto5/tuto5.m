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

wp1M = Waypoint();
wp1M.label = 'wp1M';

wp1H = Waypoint();
wp1H.label = 'wp1H';

wp2  = Waypoint();
wp2.label = 'wp2';

wp3  = Waypoint();
wp3.label = 'wp3';

wp4  = Waypoint();
wp4.label = 'wp4';

wp5  = Waypoint();
wp5.label = 'wp5';

wp6H = Waypoint();
wp6H.label = 'wp6H';

wp6M = Waypoint();
wp6M.label = 'wp6M';

wp6L = Waypoint();
wp6L.label = 'wp6L';

% take off / landing positions
wp1L.SetPosition(vp1+[0 0 0.25]);
wp6L.SetPosition(vp2+[0 0 0.25]);

% take off / landing approach positions
wp1M.SetPosition(vp1+[0 0 1]);
wp6M.SetPosition(vp2+[0 0 1]);

% take off / landing hovering positions
wp1H.SetPosition(wp1L.Position);
wp1H.z = 60;
wp6H.SetPosition(wp6L.Position);
wp6H.z = wp1H.z;

wp1H.SetPosition(wp1H.Position + 2 * wp1H.DirectionTo(wp6H));
wp3.SetPosition(wp6H.Position - 10 * wp1H.DirectionTo(wp6H));

% route             
wp2.SetPosition(wp1H.Position + [ 50  50 0]);
wp3.SetPosition(wp2.Position  + [ 50   0 0]);
wp4.SetPosition(wp3.Position  + [  0 -50 0]);
wp5.SetPosition(wp4.Position  + [-50 -50 0]);

% Compose the flight plan
fp1  = FlightPlan(Waypoint.empty);
fp1.mode = InterpolationModes.TP;

fp1.InsertWaypoint(wp1L);
fp1.InsertWaypoint(wp1M);
fp1.InsertWaypoint(wp1H);
fp1.InsertWaypoint(wp2);
fp1.InsertWaypoint(wp3);
fp1.InsertWaypoint(wp4);
fp1.InsertWaypoint(wp5);
fp1.InsertWaypoint(wp6H);
fp1.InsertWaypoint(wp6M);
fp1.InsertWaypoint(wp6L);

% waypoint time intervals
fp1.SetTimeFromVel('wp1M',0.2);
fp1.SetTimeFromVel('wp1H',info.maxVerticalVel);
fp1.SetTimeFromVel('wp2' ,info.maxForwardVel-2);
fp1.SetTimeFromVel('wp3' ,info.maxForwardVel-2);
fp1.SetTimeFromVel('wp4' ,info.maxForwardVel-2);
fp1.SetTimeFromVel('wp5' ,info.maxForwardVel-2);
fp1.SetTimeFromVel('wp6H',info.maxForwardVel-2);
fp1.SetTimeFromVel('wp6M',info.maxVerticalVel);
fp1.SetTimeFromVel('wp6L',0.2);

fp1.PositionFigure("FP1: POSITION",0.1);
fp1.VelocityFigure("FP1: VELOCITY",0.1);


fp2 = fp1.Convert2TPV();

fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);

fp2.ApplyDubinsAt('wp3',info.maxAngularVel);
 
% fp2 = fp1.Convert2TP(0.1);
 


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

% Deploy fleet
%               x        y        z       rz
fleetLoc = [ -190.00  -119.00  +048.10   -pi/2  ];
for i = 1:size(fleetLoc,1)
    id = sprintf('UAV%02d', i);
    operator.DeployUAV(info,id, ...
        fleetLoc(i,1:3), ...
        [0 0 fleetLoc(i,4)]);
    monitor.TrackUAV(id);
end

% Comunicate Flight Plans
time = operator.GetTime();
fp1.RescheduleAt(time + 10);
operator.SendFlightPlan('UAV01',fp1);
operator.WaitTime(fp1.FinishTime);

% Display
monitor.PositionFigure('UAV01',fp1,0.01);
monitor.VelocityFigure('UAV01',fp1,0.01);

%

operator.PauseSim;



