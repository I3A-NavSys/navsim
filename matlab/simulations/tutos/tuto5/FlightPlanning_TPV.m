clc
clear
run('../../../tools/NAVSIM_PATHS');



% -------------
%Create a Flight Plan for the drone 1

%              t      x        y        z       vx    vy    vz
way_data1 = [ 05   -190.00  -119.00  +048.05   0.00  0.00  0.00
              10   -190.00  -119.00  +052.00   0.00  0.00  0.00   
              28   -190.00  -154.00  +052.00   0.00 -2.00  0.00
              32   -185.00  -159.00  +052.00   2.00  0.00  0.00
              48   -155.00  -159.00  +052.00   2.00  0.00  0.00
              52   -150.00  -154.00  +052.00   0.00  2.00  0.00
              68   -150.00  -124.00  +052.00   0.00  2.00  0.00
              72   -155.00  -119.00  +052.00  -2.00  0.00  0.00
              90   -190.00  -119.00  +052.00   0.00  0.00  0.00
              95   -190.00  -119.00  +048.05   0.00  0.00  0.00   ];








fp1  = FlightPlan(Waypoint.empty);
for i = 1:size(way_data1,1)
    wp = Waypoint();
    wp.t = way_data1(i,1);
    wp.SetPosition(way_data1(i,2:4));
    wp.SetVelocity(way_data1(i,5:7));
    fp1.SetWaypoint(wp);
end
fp1.mode = "TPV0";
fp1.PositionFigure("FP1: POSITION",0.1);
fp1.VelocityFigure("FP1: VELOCITY",0.1);

fp2 = fp1.Convert2TP(0.1);
fp2.PositionFigure("FP2: POSITION",0.1);
fp2.VelocityFigure("FP2: VELOCITY",0.1);




builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
monitor  = SimpleMonitor('monitor');


% -------------
% Set vertiports

%               x        y        z       rz
portsLoc = [ -190.00  -119.00  +048.00    pi/2
             -152.00  -106.00  +049.00    pi/2
             +180.00  +033.00  +050.00    00
           ];


for i = 1:size(portsLoc,1)
   
    id = sprintf('BASE%02d', i);
    builder.DeployModel('UAM/vertiport_H', id, ...
        portsLoc(i,1:3), ...
        [0 0 portsLoc(i,4)]);
    operator.SetVertiport(id,portsLoc(i,1:3),1);

end





% -------------
% Deploy fleet

%               x        y        z       rz
fleetLoc = [ -190.00  -119.00  +048.10   -pi/2
             -152.00  -106.00  +049.10    pi/2
             +180.00  +033.00  +050.10   -pi*0.9
           ];


info = UAVinfo('',UAVmodels.MiniDroneFP1);
info.maxForwardVel = 10;

for i = 1:size(fleetLoc,1)
    id = sprintf('UAV%02d', i);
    operator.DeployUAV(info,id, ...
        fleetLoc(i,1:3), ...
        [0 0 fleetLoc(i,4)]);
    monitor.TrackUAV(id);
end






%Comunicate Flight Plans
operator.ResetSim;
pause(0.1)
operator.SendFlightPlan('UAV01',fp2);
operator.WaitTime(fp2.FinishTime);

% Display
monitor.PositionFigure('UAV01',fp1,0.1);
monitor.VelocityFigure('UAV01',fp1,0.1);

%%

operator.PauseSim;



