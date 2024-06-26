
clear builder operator monitor;
clc

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

% -------------
% UAV performance
info = UAVinfo('',UAVmodels.MiniDroneFP1);
info.maxForwardVel   = 12;    % m/s
info.maxForwardAcel  =  1;    % m/s2
info.maxVerticalVel  =  3;    % m/s  
info.maxVerticalAcel =  1;    % m/s2
info.maxAngularVel   =  5;    % rad/s
info.maxAngularAcel  =  1;    % rad/s2


% Deploy UAVs
operator.DeployUAV(info,'UAV01', ...
    [ -190.00  -119.00  +048.10 ],...
    [    0.00     0.00     pi/4  ]);
monitor.TrackUAV('UAV01');

operator.DeployUAV(info,'UAV02', ...
    [ -189.50  -119.00  +048.10 ],...
    [    0.00     0.00     pi/4 ]);
monitor.TrackUAV('UAV02');



time = operator.GetTime();

fp1.RescheduleAt(time + 10);
operator.SendFlightPlan('UAV01',fp1);

pause(0.2);

% fp2.RescheduleAt(time + 10.2);
% % fp3 = fp2.Convert2TP(0.25);
% operator.SendFlightPlan('UAV02',fp2);


% operator.WaitTime(max(fp1.FinishTime,fp2.FinishTime));
operator.WaitTime(fp1.FinishTime);
monitor.PositionFigure('UAV01',fp1,0.01);
monitor.VelocityFigure('UAV01',fp1,0.01);
% monitor.PositionFigure('UAV02',fp2,0.01);
% monitor.VelocityFigure('UAV02',fp2,0.01);


operator.PauseSim;
