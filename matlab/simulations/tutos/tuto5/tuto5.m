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

% -------------
% Flight Plan
%                x        y        z   
way_data = [ -190.00  -119.00  +048.05
             -152.00  -106.00  +049.05  ];


fp1  = FlightPlan(Waypoint.empty);
for i = 1:size(way_data,1)
    wp = Waypoint();
    wp.t = way_data(i,1);
    wp.SetPosition(way_data(i,2:4));
    wp.SetVelocity(way_data(i,5:7));
    fp1.SetWaypoint(wp);
end




fp1.mode = "TPV0";
fp1.PositionFigure("FP1: POSITION",0.1);
fp1.VelocityFigure("FP1: VELOCITY",0.1);




fp2 = fp1.Convert2TP(0.1);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);




% -------------
% Simulation
builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
monitor  = SimpleMonitor('monitor');

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
operator.ResetSim;
pause(0.1)
operator.SendFlightPlan('UAV01',fp2);
operator.WaitTime(fp2.FinishTime);

% Display
monitor.PositionFigure('UAV01',fp1);
monitor.VelocityFigure('UAV01',fp1);

%%

operator.PauseSim;



