
% Set ROS2 connectors 

clear builder operator monitor;
clc

builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
pause(0.1)
operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
pause(0.1)
monitor  = SimpleMonitor('monitor');
pause(0.1)


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
end


% -------------
% Deploy UAVs

%               x        y        z       rz
fleetLoc = [ -190.00  -119.00  +048.10    pi/2
             -152.00  -106.00  +049.10    pi/2
             +180.00  +033.00  +050.10    -0.9*pi
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


% -------------
% Comunicate Flight Plans
operator.ResetSim;
operator.SendFlightPlan('UAV01',fp1);
operator.SendFlightPlan('UAV02',fp2);
operator.SendFlightPlan('UAV03',fp3);



% -------------
%Wait Flight Plans execution
operator.WaitTime(max([fp1.FinishTime fp2.FinishTime fp3.FinishTime]));
% operator.RemoveUAV('UAV01');
% operator.RemoveUAV('UAV02');
% operator.RemoveUAV('UAV03');
operator.PauseSim;


% -------------
% Show results
monitor.PositionFigure('UAV01',fp1,0.1);
monitor.VelocityFigure('UAV01',fp1,0.1);
[medE,maxE,t] = monitor.PathFollowingError('UAV01',fp1);

monitor.PositionFigure('UAV02',fp2,0.1);
monitor.VelocityFigure('UAV02',fp2,0.1);

monitor.PositionFigure('UAV03',fp3,0.1);
monitor.VelocityFigure('UAV03',fp3,0.1);

